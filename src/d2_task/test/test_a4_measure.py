"""W144 A4(E-119) 놓은 블록 다시 재기 — 기본 꺼짐 · 켜면 VERIFY 통과 뒤 방금 놓은 블록만 재서 기록만 한다. ROS · 로봇 없음."""
import json
import math

import pytest

from d2_task.task_manager import TaskManager, placed_offset
from test_task_manager import CFG, IDS, SAFE_OK, SAFE_STOP, FakeIO, drive


def ok_body(part_id, blocks, rejected=None, detail='', stamp=1791960000.123):
    """비전 #177 응답 꼴(성공 · 측정 거절 · 블록 없음 — 모두 ok true)."""
    return {'ok': True, 'part_id': part_id, 'blocks': blocks, 'rejected': rejected, 'detail': detail, 'measured_stamp': stamp}


class MeasureIO(FakeIO):
    """FakeIO + w144_measure(script: 'off' = None, 'ok' = 설계 중심 + (1.5, −2.0) mm, 그 밖 = 그대로 돌려줄 값) · w144_event 기록."""

    def __init__(self, script='ok'):
        super().__init__()
        self.script, self.measured, self.w144_events, self.design_ids = script, [], [], []

    def w144_measure(self, run_id, design_id, part_id, should_abort):
        self.events.append(('call', 'measure', part_id))
        self.measured.append((run_id, part_id))
        self.design_ids.append(design_id)
        if self.script == 'off':
            return None
        if self.script == 'ok':
            c = next(p for p in self.manager.planner.parts if p['part_id'] == part_id)['center']
            return True, '', ok_body(part_id, [{'x_m': c[0] + 0.0015, 'y_m': c[1] - 0.002, 'top_z_m': c[2], 'yaw_deg': 1.0,
                                                'up': 'THICKNESS', 'confidence': 1.0}])
        return self.script

    def w144_event(self, event, **fields):
        self.w144_events.append((event, fields))


def run(io, **scripts):
    """bench 를 끝까지(DONE) 돌린다. CSV 기록은 rows 로 모은다."""
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    rows = []
    m.logger.log = lambda kind, part_id=None, value=None, module='task': rows.append((kind, part_id, value))
    for name, value in scripts.items():
        setattr(io, name, value)
    assert m.command('select_design', 'bench') == (True, '') and m.command('start') == (True, '')
    drive(m, 'DONE')
    return m, rows


def without_measure(calls):
    return [c for c in calls if c[:2] != ('call', 'measure')]


def test_꺼져_있으면_기존_호출과_같고_기록도_없다():
    base = FakeIO()
    run(base)
    off = MeasureIO('off')
    _, rows = run(off)
    assert without_measure(off.events) == base.events          # 측정 말고는 호출 · 상태 순서가 그대로
    assert not [r for r in rows if r[0] == 'a4_measure'] and 'placed_measured' not in [e for e, _ in off.w144_events]


def test_켜면_VERIFY_통과한_블록마다_그_블록만_한번_재서_기록한다():
    io = MeasureIO('ok')
    m, rows = run(io)
    assert [p for _, p in io.measured] == IDS and all(r for r, _ in io.measured)        # 놓은 차례대로 · run_id 를 실어 부름
    assert set(io.design_ids) == {m.design_id} and m.design_id                            # design_id 필수(#177) — 블록 이름에서 잘라 내지 않음
    rec = [json.loads(v) for k, _, v in rows if k == 'a4_measure']
    assert len(rec) == len(IDS) and all({k: r[k] for k in ('status', 'dx_mm', 'dy_mm', 'yaw_deg', 'up', 'measured_stamp')} ==
                                        {'status': 'MEASURED', 'dx_mm': 1.5, 'dy_mm': -2.0, 'yaw_deg': 1.0, 'up': 'THICKNESS',
                                         'measured_stamp': 1791960000.123} for r in rec)
    assert rec[0]['target_x_m'] is not None and abs(rec[0]['x_m'] - rec[0]['target_x_m'] - 0.0015) < 1e-6   # 잰 값 · 비교한 목표 둘 다 남김
    assert [f['part_id'] for e, f in io.w144_events if e == 'placed_measured'] == IDS
    assert io.world == set(IDS)                                  # 기록만 — 다시 집기 · 상태 변화 없음


def test_측정이_실패해도_조립은_이어가고_오차_0으로_적지_않는다():
    io = MeasureIO((False, 'SCAN_FAILED', {'ok': False, 'reason': 'SCAN_FAILED', 'detail': '요청 오류: 설계에 없는 part_id'}))
    _, rows = run(io)
    rec = [json.loads(v) for k, _, v in rows if k == 'a4_measure']
    assert len(rec) == len(IDS) and all(r == {'status': 'UNMEASURED', 'reason': 'SCAN_FAILED', 'detail': '요청 오류: 설계에 없는 part_id',
                                              'measured_stamp': None} for r in rec)
    assert io.world == set(IDS)


def test_VERIFY가_실패한_블록은_재지_않는다():
    io = MeasureIO('ok')
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    io.check_script = [None, {IDS[0]: 'absent'}]               # 선택 때 진행 확인은 정상, 첫 VERIFY 에서 놓은 블록이 없음(OFFSET_OVER)
    assert m.command('select_design', 'bench') == (True, '') and m.command('start') == (True, '')
    drive(m, 'ERROR')
    assert io.states[-1]['message_id'] == 'offset_over' and io.measured == []


C = [0.40, -0.05, 0.02]


P = 'P1'


def U(reason, detail, stamp=None):
    return {'status': 'UNMEASURED', 'reason': reason, 'detail': detail, 'measured_stamp': stamp}


ST = 1791960000.123


@pytest.mark.parametrize('res, want', [
    ((True, '', ok_body(P, [], 'NO_PART', '윗면 점 거의 없음')), U('NO_PART', '윗면 점 거의 없음', ST)),               # 블록 없음
    ((True, '', ok_body(P, [], 'NEIGHBOR_MERGED', '옆 블록과 붙음')), U('NEIGHBOR_MERGED', '옆 블록과 붙음', ST)),     # 측정 거절
    ((False, 'SCAN_FAILED', {'ok': False, 'reason': 'SCAN_FAILED', 'detail': '요청 오류: design_id 없음'}),
     U('SCAN_FAILED', '요청 오류: design_id 없음')),                                                                  # 요청 오류
    ((True, '', ok_body(P, [], None)), U('BAD_RESPONSE', 'blocks [] 인데 rejected 코드 없음', ST)),
    ((True, '', ok_body('P2', [{'x_m': 0.4, 'y_m': 0.0}])), U('BAD_RESPONSE', "ok · part_id 가 요청과 다름('P2')", ST)),
    ((True, '', ok_body(P, [{'x_m': 0.4, 'y_m': 0.0}], 'FEW_POINTS')), U('BAD_RESPONSE', "blocks 1개 · rejected 'FEW_POINTS'", ST)),
    ((True, '', ok_body(P, [{'x_m': 0.4, 'y_m': 0.0}, {'x_m': 0.4, 'y_m': 0.0}])), U('BAD_RESPONSE', 'blocks 2개 · rejected None', ST)),
    ((True, '', ok_body(P, [{'x_m': math.nan, 'y_m': 0.0}])), U('BAD_RESPONSE', 'x_m · y_m 없음 · 유한한 수 아님', ST)),
    ((True, '', ok_body(P, [{'y_m': 0.0}])), U('BAD_RESPONSE', 'x_m · y_m 없음 · 유한한 수 아님', ST)),
    ((True, '', ok_body(P, [{'x_m': 10 ** 400, 'y_m': 0.0}])), U('BAD_RESPONSE', 'x_m · y_m 없음 · 유한한 수 아님', ST)),
    ((True, '', ok_body(P, [{'x_m': '0.4', 'y_m': 0.0}])), U('BAD_RESPONSE', 'x_m · y_m 없음 · 유한한 수 아님', ST)),
    ((True, '', ok_body(P, [{'x_m': True, 'y_m': 0.0}])), U('BAD_RESPONSE', 'x_m · y_m 없음 · 유한한 수 아님', ST)),
    ((True, '', ok_body(P, [], 'COVERED', stamp='어제')), U('COVERED', '', None)),                                    # 시각이 이상하면 None
    ((False, 'SCAN_FAILED', {'ok': False, 'detail': {'code': 'X'}}), U('SCAN_FAILED', None)),
    ((False, 'TIMEOUT', None), U('TIMEOUT', None)),
    ((False, '', None), U('ERROR', None)),
])
def test_못_잰_답은_UNMEASURED_로_오차를_만들지_않는다(res, want):
    assert placed_offset(C, P, *res) == want


def test_잰_답은_설계_중심과의_차를_mm_로():
    r = placed_offset(C, P, True, '', ok_body(P, [{'x_m': 0.403, 'y_m': -0.0525, 'yaw_deg': -2.0, 'up': 'THICKNESS'}]))
    assert r == {'status': 'MEASURED', 'x_m': 0.403, 'y_m': -0.0525, 'target_x_m': 0.4, 'target_y_m': -0.05,
                 'dx_mm': 3.0, 'dy_mm': -2.5, 'yaw_deg': -2.0, 'up': 'THICKNESS', 'tilted': None, 'top_z_m': None, 'measured_stamp': ST}


def test_측정_호출이_예외여도_조립은_이어가고_못_잰_것으로_적는다():
    class Boom(MeasureIO):
        def w144_measure(self, run_id, design_id, part_id, should_abort):
            self.measured.append((run_id, part_id))
            raise RuntimeError('비전 노드 없음')
    io = Boom()
    _, rows = run(io)
    rec = [json.loads(v) for k, _, v in rows if k == 'a4_measure']
    assert len(rec) == len(IDS) and rec[0] == {'status': 'UNMEASURED', 'reason': 'ERROR', 'detail': 'RuntimeError: 비전 노드 없음', 'measured_stamp': None}
    assert io.world == set(IDS)


def test_yaw_up_tilted_top_z_m_이_이상하면_None():
    for bad in (math.inf, 10 ** 400, True, '3.0'):                # 무한 · 아주 큰 정수(OverflowError) · bool · 글자
        r = placed_offset(C, P, True, '', ok_body(P, [{'x_m': 0.4, 'y_m': -0.05, 'yaw_deg': bad, 'up': 'SIDEWAYS', 'top_z_m': bad, 'tilted': 'no'}]))
        assert r['status'] == 'MEASURED' and r['yaw_deg'] is None and r['up'] is None and r['top_z_m'] is None and r['tilted'] is None


def _on(**kw):
    """측정을 켠 TaskManager — CHECK 로 출발까지만(블록마다 측정 fn 을 시험이 정함)."""
    io = MeasureIO(kw.pop('script', 'ok'))
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    rows = []
    m.logger.log = lambda kind, part_id=None, value=None, module='task': rows.append((kind, part_id, value))
    assert m.command('select_design', 'bench') == (True, '') and m.command('start') == (True, '')
    return m, io, rows


def test_측정_시간초과는_기존_TIMEOUT_경로로_정지요청_뒤_정지확인_대기():
    m, io, rows = _on(script=(False, 'TIMEOUT', None))
    io.stop_script = [(True, '접수')]
    drive(m, 'ERROR')
    assert ('stop', 'TIMEOUT') in io.calls and m._timeout_pending
    assert io.states[-1]['message_id'] == 'stop_pending' and 'SELECT' not in [s['state'] for s in io.states[-3:]]
    rec = [json.loads(v) for k, _, v in rows if k == 'a4_measure']
    assert rec == [{'status': 'UNMEASURED', 'reason': 'TIMEOUT', 'detail': None, 'measured_stamp': None}]
    assert len(io.measured) == 1 and len(io.world) == 1          # 다음 블록으로 가지 않았다


def test_측정_중_정지면_답을_버리고_다음_단계로_가지_않는다():
    m, io, rows = _on()

    def stop_during(run_id, design_id, part_id, should_abort):
        io.measured.append((run_id, part_id))
        m.on_safety(SAFE_STOP)                                    # 조회 중 정지 노드가 세움
        return True, '', ok_body(part_id, [{'x_m': 0.4, 'y_m': 0.0}])
    io.w144_measure = stop_during
    drive(m, 'STOPPED')
    assert not [r for r in rows if r[0] == 'a4_measure'] and len(io.world) == 1


def test_측정_중_run이_바뀌면_늦은_답을_새_run_에_적지_않는다():
    m, io, rows = _on()

    def run_changes(run_id, design_id, part_id, should_abort):
        io.measured.append((run_id, part_id))
        m._epoch += 1                                             # 그 사이 상태가 바뀜(새 run · 다른 단계)
        return True, '', ok_body(part_id, [{'x_m': 0.4, 'y_m': 0.0}])
    io.w144_measure = run_changes
    for _ in range(40):
        m.run_once()
        if io.measured:
            break
    assert io.measured and not [r for r in rows if r[0] == 'a4_measure']


def test_io_답_꼴이_이상하면_못_잰_것으로_적고_이어_간다():
    m, io, rows = _on(script='bad-shape')
    drive(m, 'DONE')
    rec = [json.loads(v) for k, _, v in rows if k == 'a4_measure']
    assert len(rec) == len(IDS) and rec[0]['status'] == 'UNMEASURED' and rec[0]['reason'] == 'ERROR'


# ---------- task_node: 측정 호출만 실패 본문을 넘기고, 다른 find_blocks 는 그대로 ----------
def _node(measure_on):
    """ROS 없이 TaskNode 의 _call · _scan_query · _failed_body · find_blocks · w144_measure 본문만 실행한다(test_task_timeouts.node 와 같은 방식)."""
    import ast
    import threading
    import time
    from pathlib import Path
    from types import SimpleNamespace
    from d2_task.task_manager import wait_until
    path = Path(__file__).parents[1] / 'd2_task/task_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskNode')
    names = {'_call', '_scan_query', '_failed_body', 'find_blocks', 'w144_measure', 'w144_regrasp_on'}
    cls = ast.ClassDef(name='TaskNode', bases=[], keywords=[], decorator_list=[],
                       body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names])
    scope = dict(threading=threading, time=time, json=json, wait_until=wait_until, JsonQuery=SimpleNamespace(Request=SimpleNamespace))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), scope)
    n = scope['TaskNode']()
    n.service_s = n.find_blocks_s = 0.2
    n.get_parameter = lambda name: SimpleNamespace(value=measure_on)
    return n


class _Done:
    """요청을 기록하고 정해진 응답을 바로 돌려주는 가짜 서비스."""

    def __init__(self, response):
        self.response, self.requests = response, []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        from concurrent.futures import Future
        self.requests.append(json.loads(req.request_json))
        f = Future()
        f.set_result(self.response)
        return f

    def remove_pending_request(self, future):
        pass


def _failed(body_text):
    from types import SimpleNamespace
    return SimpleNamespace(success=False, reason='SCAN_FAILED', response_json=body_text)


def test_꺼짐이면_노드도_조회하지_않는다():
    n = _node(False)
    n.find_cli = _Done(_failed('{}'))
    assert n.w144_measure('R1', 'D1', 'P1', lambda: False) is None and n.find_cli.requests == []


def test_측정_호출만_실패_본문을_넘기고_요청은_assembly_design_id_part_id():
    n = _node(True)
    n.find_cli = _Done(_failed(json.dumps({'ok': False, 'reason': 'SCAN_FAILED', 'detail': '요청 오류: 설계 D1 에 없는 part_id P1'})))
    ok, why, body = n.w144_measure('R1', 'D1', 'P1', lambda: False)
    assert (ok, why, body['detail']) == (False, 'SCAN_FAILED', '요청 오류: 설계 D1 에 없는 part_id P1')
    assert n.find_cli.requests == [{'run_id': 'R1', 'area': 'assembly', 'design_id': 'D1', 'part_id': 'P1'}]
    assert n.find_blocks('R1', lambda: False) == (False, 'SCAN_FAILED', None)          # 다른 find_blocks 는 예전처럼 본문을 버림


@pytest.mark.parametrize('text', ['', '깨짐', '[1, 2]', '{"detail": NaN}', '"글자"'])
def test_실패_본문을_못_읽으면_None(text):
    n = _node(True)
    n.find_cli = _Done(_failed(text))
    assert n.w144_measure('R1', 'D1', 'P1', lambda: False) == (False, 'SCAN_FAILED', None)


def test_확인과_기록_사이에_정지가_끼어도_기록하지_않는다(monkeypatch):
    """조회가 끝나고 기록 직전(placed_offset 계산 중)에 정지가 와도 새로 적지 않는다 — 확인 · 기록이 한 잠금 안."""
    import d2_task.task_manager as tm
    m, io, rows = _on()
    real = tm.placed_offset

    def offset_then_stop(*a):
        out = real(*a)
        m.on_safety(SAFE_STOP)
        return out
    monkeypatch.setattr(tm, 'placed_offset', offset_then_stop)
    drive(m, 'STOPPED')
    assert not [r for r in rows if r[0] == 'a4_measure']
