"""10/14 점검 ③(한석형) — 정지 순서 · 정지 입력 방어 · 맞는 코드 없는 거절(E-111 A3) 시험. ROS · 로봇 없음.

노드 메서드는 test_task_timeouts 와 같은 방식(ast 로 실제 본문을 꺼내 가짜 클라이언트로 실행)으로 본다.
"""
import ast
import copy
import logging
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest

from d2_task.task_manager import TaskManager
from test_task_manager import CFG, SAFE_OK, FakeIO, make
from test_task_timeouts import Client, Handle, completed, goal, node  # noqa: F401 — node 는 pytest fixture

TASK_NODE = Path(__file__).parents[1] / 'd2_task/task_node.py'


# ---------- 정지 입력 방어 ----------
@pytest.mark.parametrize('bad', ['stopped', None, [], 3, True])
def test_safety_state_가_객체가_아니면_버리고_작업은_계속_돈다(bad):
    m, io = make()
    before = (m.safety, m.safety_count, m._stop_count)
    m.on_safety(bad)
    assert (m.safety, m.safety_count, m._stop_count) == before
    m.run_once()                                         # 예전에는 halted() 의 .get 에서 터졌다
    assert m.state == 'IDLE'


# ---------- Ctrl+C: 정지 요청이 목표 취소보다 먼저 ----------
def test_집기_결과를_기다리다_CtrlC_면_정지_요청_뒤에_취소(node):
    events = []
    handle = Handle()
    node.pick_cli = SimpleNamespace(send_goal_async=lambda g: completed(handle))
    node.request_stop = lambda why: events.append(('stop', why)) or (True, '')
    node.cancel_active = lambda: events.append(('cancel',))
    calls = {'n': 0}

    def abort():
        calls['n'] += 1
        if calls['n'] > 3:                               # 수락 뒤 결과를 기다리는 중에 Ctrl+C
            raise KeyboardInterrupt
        return False
    with pytest.raises(KeyboardInterrupt):
        node.pick_place(goal(), abort)
    assert events[:2] == [('stop', 'CTRL_C'), ('cancel',)]


def test_목표_수락을_기다리다_CtrlC_면_정지_요청부터(node):
    events = []
    pending = Future()                                   # 수락 답이 아직 없다
    node.pick_cli = SimpleNamespace(send_goal_async=lambda g: pending)
    node.request_stop = lambda why: events.append(('stop', why)) or (True, '')
    node._cancel_late = lambda f: events.append(('late cancel',))
    calls = {'n': 0}

    def abort():
        calls['n'] += 1
        if calls['n'] > 1:
            raise KeyboardInterrupt
        return False
    with pytest.raises(KeyboardInterrupt):
        node.pick_place(goal(), abort)
    assert events[0] == ('stop', 'CTRL_C')
    pending.set_result(Handle())                         # 늦게 받아지면 그때 취소
    assert ('late cancel',) in events


def _run_main(run_once, fail=()):
    """실제 main 본문을 가짜 노드로 실행한다. fail = 예외를 낼 단계 이름들('shutdown' · 'stop' · 'cancel'). 반환: (events, 던진 예외)."""
    tree = ast.parse(TASK_NODE.read_text(encoding='utf-8'))
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    events = []
    def step(name, result=None):
        def run(*args):
            events.append(name if not args else (name, args[0]))
            if name in fail:
                raise OSError(f'{name} 고장')
            return result
        return run
    manager = SimpleNamespace(run_once=run_once, shutdown=step('shutdown'),
                              finalize=lambda _s: events.append('finalize'))
    logger = SimpleNamespace(info=lambda _s: None, error=lambda s: events.append(('log', s[:6])), warn=lambda _s: None)
    task = SimpleNamespace(manager=manager, get_logger=lambda: logger,
                           cancel_active=step('cancel'),
                           request_stop=step('stop', (True, '')),
                           destroy_node=lambda: events.append('destroy'),
                           w144_finish=lambda _done: events.append('w144 finish'))
    executor = SimpleNamespace(add_node=lambda _n: None, spin=lambda: None,
                               shutdown=lambda **_kw: events.append('executor shutdown'))
    scope = dict(logging=logging, init_ros=lambda: None, TaskNode=lambda: task,
                 MultiThreadedExecutor=lambda: executor, threading=threading,
                 rclpy=SimpleNamespace(ok=lambda: True, shutdown=lambda: events.append('ros shutdown')),
                 time=time, LOOP_S=0, LOG_FLUSH_S=0)
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(TASK_NODE), 'exec'), scope)
    try:
        scope['main']()
    except Exception as e:                               # noqa: BLE001
        return events, e
    return events, None


def test_main_예상_못_한_예외면_정지_요청_STOP_TASK_뒤_취소하고_다시_던진다():
    def boom():
        raise RuntimeError('고장')
    events, err = _run_main(boom)
    assert isinstance(err, RuntimeError)
    steps = [e for e in events if not (isinstance(e, tuple) and e[0] == 'log')]
    assert steps == ['shutdown', ('stop', ''), 'cancel', 'finalize', 'w144 finish', 'executor shutdown', 'destroy', 'ros shutdown']


@pytest.mark.parametrize('fail', [('shutdown',), ('stop',), ('cancel',), ('shutdown', 'stop', 'cancel')])
def test_main_예외_때_한_단계가_실패해도_다음_단계와_마무리를_시도하고_원래_예외를_던진다(fail):
    def boom():
        raise RuntimeError('고장')
    events, err = _run_main(boom, fail)
    assert isinstance(err, RuntimeError) and str(err) == '고장'     # 단계 실패가 원래 예외를 덮지 않는다
    steps = [e for e in events if not (isinstance(e, tuple) and e[0] == 'log')]
    assert steps == ['shutdown', ('stop', ''), 'cancel', 'finalize', 'w144 finish', 'executor shutdown', 'destroy', 'ros shutdown']
    logs = [e[1] for e in events if isinstance(e, tuple) and e[0] == 'log']
    marks = {'shutdown': '종료 ① ', 'stop': '종료 ② ', 'cancel': '종료 ③ '}
    assert all(any(l.startswith(marks[f]) for l in logs) for f in fail)   # 단계별로 구분된 실패 로그


def test_main_CtrlC_순서는_그대로():
    def interrupt():
        raise KeyboardInterrupt
    events, err = _run_main(interrupt)
    assert err is None
    assert events == ['shutdown', ('stop', 'CTRL_C'), 'cancel', 'finalize', 'w144 finish', 'executor shutdown', 'destroy', 'ros shutdown']


# ---------- 스캔 시작 점검에 정지 서비스 ----------
def _services_ready_node(stop_ready):
    tree = ast.parse(TASK_NODE.read_text(encoding='utf-8'))
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskNode')
    cls = ast.ClassDef(name='TaskNode', bases=[], keywords=[],
                       body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name == 'services_ready'],
                       decorator_list=[])
    scope = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(TASK_NODE), 'exec'), scope)
    n = scope['TaskNode']()
    n.move_cli = n.capture_cli = n.infer_cli = Client()
    n.stop_cli = Client(ready=stop_ready)
    return n


def test_스캔_시작_점검은_정지_서비스가_없으면_막는다():
    assert '/d2/safety/stop' in _services_ready_node(False).services_ready(scan=True)
    assert _services_ready_node(True).services_ready(scan=True) == []


def test_정지_서비스가_없으면_스캔을_시작하지_않는다():
    m, io = make()
    io.missing = ['/d2/safety/stop']
    io.services_ready = lambda scan=False: list(io.missing)
    ok, reason = m.command('scan')
    assert ok is False and m.state == 'IDLE'


# ---------- E-111 A3: 맞는 코드 없는 거절 = 빈 값 + 글 ----------
def test_scan_poses_설정_오류는_빈_reason():
    cfg = copy.deepcopy(CFG)
    cfg['scan']['poses'] = ['no_such_pose']
    io = FakeIO()
    m = TaskManager(cfg, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    assert m.command('scan') == (False, '')
    assert 'scan.poses' in io.states[-1]['message'] and m.state == 'IDLE'


@pytest.mark.parametrize('why,expected', [('ERROR', ''), ('NOT_FOUND', ''), ('', ''), ('TIMEOUT', 'TIMEOUT'),
                                          ('STOPPED', 'STOPPED'), ('BUSY', 'BUSY')])
def test_설계_조회_실패는_TIMEOUT_STOPPED_BUSY_만_코드_나머지는_빈_값(why, expected):
    m, io = make()
    io.get_design = lambda design_id, should_abort: (False, why, None)
    ok, reason = m.command('select_design', 'bench')
    assert ok is False and reason == expected
    assert '설계를 못 가져왔다' in io.states[-1]['message'] and m.state == 'IDLE'
