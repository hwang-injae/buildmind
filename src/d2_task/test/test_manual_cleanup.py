"""W183 수동 정리(10/13 16:30 PL) — STOPPED 의 화면 cancel · gripper_open 시험. ROS · 로봇 없음.

STOPPED cancel = 사람이 정리했다는 확인 → 기록 STOPPED 로 닫고 조립을 비우되 상태는 STOPPED, 잠금 · 팔은 그대로.
그 뒤 resume(잠금이 풀린 safety/state) → 기존 RECOVER 비조립 경로로 IDLE. gripper_open 은 STOPPED 에서만 그리퍼를 연다.
"""
from test_task_manager import CFG, IDS, OK, SAFE_OK, SAFE_STOP, FakeIO, drive, seen
from d2_task.task_manager import GRIPPER_OPEN_M, TaskManager


class GripperIO(FakeIO):
    """FakeIO + 그리퍼 열기 가짜. gripper_script 는 호출마다 꺼내 쓰는 결과(없으면 성공)."""

    def __init__(self):
        super().__init__()
        self.gripper_script = []

    def gripper_open(self, width_m, force_n, on_done):
        """script 원소: 'accept'(기본) = 보냄, 답은 io.finish 로 / ('reject', why) = 못 보냄 / 함수 = 예외 등."""
        self.events.append(('call', 'gripper', width_m, force_n))
        step = self.gripper_script.pop(0) if self.gripper_script else 'accept'
        if callable(step):
            return step(self)
        if step == 'accept':
            self.pending_done = on_done
            return True, ''
        return False, step[1]

    def finish(self, ok, why=''):
        """그리퍼 서버의 (늦은) 실제 답."""
        self.pending_done(ok, why)


def stopped_mid_build(**scripts):
    """bench 를 출발해 블록 하나를 놓은 뒤 두 번째 집기에서 정지 노드가 세운 STOPPED 를 만든다."""
    io = GripperIO()
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    for name, value in scripts.items():
        setattr(io, name, value)
    io.pick_script = [OK, (False, 'STOPPED')]
    assert m.command('select_design', 'bench') == OK and m.command('start') == OK
    drive(m, 'STOPPED')
    m.on_safety(SAFE_STOP)                               # 잠긴 상태
    m.on_gripper({'grasped': True})                      # 쥔 채로 멈춘 경우 — 기존 조립 복구라면 ERROR 로 갈 상황
    return m, io


def moves(io):
    return [c for c in io.calls if c[0] in ('move_to', 'pick')]


def test_STOPPED_cancel_은_기록을_닫고_조립을_비우되_STOPPED_와_잠금을_유지한다():
    m, io = stopped_mid_build()
    before = moves(io)
    assert m.command('cancel') == OK
    assert m.state == 'STOPPED' and m.safety['locked']
    assert m.planner is None and m.design_id is None and m.goal is None and m._assembling is False
    assert m.last_build['result'] == 'STOPPED' and m.last_build['placed'] == 1
    for _ in range(5):
        m.on_safety(SAFE_STOP)                           # 정리 중 잠금 상태 갱신이 와도
        m.run_once()
    assert m.state == 'STOPPED' and moves(io) == before  # 팔 이동 · 집기 호출 없음


def test_정리_뒤_resume_은_RECOVER_비조립_경로로_IDLE_이동_없음():
    m, io = stopped_mid_build()
    before = moves(io)
    assert m.command('cancel') == OK
    m.on_safety(SAFE_OK)                                 # 화면 [정리 끝] → safety/resume
    drive(m, 'IDLE')
    assert 'RECOVER' in seen(io) and 'CHECK' not in seen(io)[seen(io).index('RECOVER'):]
    assert moves(io) == before and m.run_id is None
    for _ in range(5):
        m.run_once()
    assert m.state == 'IDLE' and moves(io) == before     # 자동 재개 없음


def test_반복_cancel_은_새_기록이나_종료를_더_만들지_않는다():
    m, io = stopped_mid_build()
    assert m.command('cancel') == OK
    first = m.last_build
    assert m.command('cancel') == OK
    assert m.last_build is first and m.state == 'STOPPED' and len(m.pending_builds) <= 1


def test_정지_결과를_처리하는_단계_중의_cancel_은_BUSY_뒤에는_받는다():
    seen_reply = []

    def late(io):                                        # pick_place 가 늦게 끝나는 사이에 정지 + 정리 확인이 온다
        io.manager.on_safety(SAFE_STOP)
        io.manager._apply_safety()
        seen_reply.append(io.manager.command('cancel'))
        return (True, '')                                # 늦게 온 성공 결과
    io = GripperIO()
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    io.pick_script = [late]
    assert m.command('select_design', 'bench') == OK and m.command('start') == OK
    drive(m, 'STOPPED')
    assert seen_reply == [(False, 'BUSY')] and m.planner is not None
    before = moves(io)
    assert m.command('cancel') == OK and m.state == 'STOPPED'
    m.on_safety(SAFE_OK)
    drive(m, 'IDLE')
    assert moves(io) == before


def ids(io):
    return [st['message_id'] for st in io.states]


def test_gripper_open_은_STOPPED_에서_접수_즉시_응답하고_복구_절차_폭과_힘으로_보낸다():
    m, io = stopped_mid_build()
    assert m.command('gripper_open') == OK                       # 접수 성공 — 그리퍼가 다 열렸다는 뜻이 아니다
    assert [c for c in io.calls if c[0] == 'gripper'] == [('gripper', GRIPPER_OPEN_M, CFG['gripper']['force_n'])]
    assert GRIPPER_OPEN_M == 0.0865 and CFG['gripper']['force_n'] == 10.0
    assert m._gripper_busy and ids(io)[-1] == 'gripper_opening' and 'gripper_done' not in ids(io)
    assert m.state == 'STOPPED' and m.safety['locked']


def test_대기_중_모든_state2_는_gripper_opening_거절_cancel_resume_IDLE_에도_덮이지_않는다():
    m, io = stopped_mid_build()
    assert m.command('gripper_open') == OK
    mark = len(io.states)
    assert m.command('gripper_open') == (False, 'BUSY')          # 반복 open
    assert m.command('cancel') == OK                             # 정리 확인(알림 stopped 자리)
    before = moves(io)
    m.on_safety(SAFE_OK)                                         # resume → RECOVER(비조립) → IDLE
    drive(m, 'IDLE')
    for cmd in ('select_design', 'start', 'scan'):
        assert m.command(cmd, 'bench') == (False, 'BUSY')
    published = io.states[mark:]
    assert published and all(st['message_id'] == 'gripper_opening' for st in published)
    assert {st['state'] for st in published} >= {'STOPPED', 'RECOVER', 'IDLE'}
    assert moves(io) == before and m._gripper_busy
    io.finish(True)                                              # 실제 완료
    assert ids(io)[-1] == 'gripper_done' and io.states[-1]['message'] == '그리퍼를 열었어요' and not m._gripper_busy
    io.finish(True)                                              # 같은 답이 또 와도 한 번만
    assert ids(io).count('gripper_done') == 1
    m.on_gripper({'grasped': False})
    assert m.command('select_design', 'bench') == OK and io.states[-1]['message_id'] != 'gripper_opening'


def test_늦은_실패_답은_gripper_done_실패_문구로_접수_성공과_구분된다():
    m, io = stopped_mid_build()
    assert m.command('gripper_open') == OK
    io.finish(False, 'NO_FEEDBACK')
    assert not m._gripper_busy and ids(io)[-1] == 'gripper_done'
    assert io.states[-1]['message'] == '그리퍼 열기 실패: NO_FEEDBACK' and m.state == 'STOPPED'


def test_보내지_못하면_바로_실패_응답과_gripper_done_대기_없음():
    def boom(io):
        raise RuntimeError('통신')
    m, io = stopped_mid_build(gripper_script=[('reject', 'GRIPPER_NO_RESPONSE'), boom])
    planner = m.planner
    assert m.command('gripper_open') == (False, 'GRIPPER_NO_RESPONSE') and not m._gripper_busy
    assert ids(io)[-2:] == ['gripper_opening', 'gripper_done'] and '실패' in io.states[-1]['message']
    assert m.command('gripper_open') == (False, 'ERROR') and not m._gripper_busy
    m.run_once()
    assert m.state == 'STOPPED' and m.safety['locked'] and m.planner is planner and 'RECOVER' not in seen(io)


def test_정리_전_resume_이면_조립_RECOVER_는_열기_답까지_CHECK_로_가지_않는다():
    m, io = stopped_mid_build()
    assert m.command('gripper_open') == OK
    before = moves(io)
    m.on_safety(SAFE_OK)
    for _ in range(10):
        m.run_once()
    assert m.state == 'RECOVER' and moves(io) == before and ids(io)[-1] == 'gripper_opening'
    m.on_gripper({'grasped': False})
    io.finish(True)
    drive(m, 'CHECK')


def test_응답이_끝내_없으면_대기를_스스로_풀지_않는다():
    """현재 동작 기록용 — 해제 방법은 미결(황인재 질문 ②)."""
    m, io = stopped_mid_build()
    assert m.command('cancel') == OK and m.command('gripper_open') == OK
    m.on_safety(SAFE_OK)
    drive(m, 'IDLE')
    for _ in range(20):
        m.run_once()
    assert m._gripper_busy and m.command('start', 'bench') == (False, 'BUSY')


def test_gripper_open_은_STOPPED_밖에서는_거절하고_보내지_않는다():
    io = GripperIO()
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    assert m.command('gripper_open') == (False, 'BUSY')
    assert m.command('select_design', 'bench') == OK
    assert m.command('gripper_open') == (False, 'BUSY')
    assert not [c for c in io.calls if c[0] == 'gripper']


def test_정리하지_않으면_기존_조립_복구_그대로():
    m, io = stopped_mid_build()
    m.on_gripper({'grasped': False})                     # 사람이 빼서 빈손
    m.on_safety(SAFE_OK)
    drive(m, 'CHECK')                                    # 기존: 조립 중 정지 → RECOVER → CHECK
    drive(m, 'DONE')
    assert io.world == set(IDS)
