# -*- coding: utf-8 -*-
"""흩뿌림 집기 준비 ScatterFlow — 목표 블록을 유지한 채 find_blocks 결과로 후보를 고르고 집기 요청을 준비한다 (ROS 없이 동작, W130).

TaskManager 의 SELECT 안에서(공개 상태를 늘리지 않고) 쓰는 계산 클래스다(supply_mode scatter 에서 TaskManager 가 부른다 — 로봇 호출은 하지 않는다).
호출 흐름(연결할 때): ① SELECT 가 TaskPlanner 에서 목표 블록(part_id · 잡기 · 놓을 자세)을 정한다 → ② begin() 으로 요청 표를 받는다 →
③ 잠금 밖에서 observe_supply 로 가서 find_blocks 를 부르고 prepare() 로 후보를 계산한다(계산만 — 상태를 바꾸지 않는다) →
④ **TaskManager 잠금 안에서** commit() 이 요청 번호 · 상태 · 정지 · 종료를 다시 확인하고 같은 구간에서 후보를 적용한다(확인과 적용 사이에 정지가 끼지 못한다) →
⑤ 실패하면 on_failure() 가 실행 단계 · 잡힘 상태로 다음 동작(REPLAN_SAME · WAIT · REOBSERVE · RECOVER · ERROR)을 알려 준다.
설정은 명시적으로 주입한다: supply_mode == 'scatter' 이고 open_width_m 이 정해져야 실행 가능한 요청을 만든다.
open_width_m 은 task 노드가 0 으로 넘긴다 — PickPlace 에서 0 은 '잡기마다 robot.yaml grasp_open_pick_m 을 쓴다'이므로 목표 블록의 잡기가 바뀌어도 폭이 맞는다.
"""
import copy
import math
import threading

from d2_motion.motion_math import TILT_DEG as NODE_TILT_DEG, column, half_height, quat_from_axes, rot_z, rpy_matrix

from d2_task.block_picker import BlockPicker, grasp_target
from d2_task.recipe_document import GRASPS

SUPPLY_SCATTER = 'scatter'
# 받침 경유 1단계에서 고르는 잡기 — 모든 잡기(one_pick 과 같음). 옆으로 선 · 선 블록은 그 자세로 받침에 세워 놓고 받침에서 눕힌다(잡기 TIP)
SPARE_GRASPS = GRASPS + ('TILT',)       # 받침으로 옮길 때 쓸 수 있는 잡기 = 정본 6가지 + 기울여 집기(10/14 점검 ③-5)
SIZE_KEYS = ('LENGTH', 'WIDTH', 'THICKNESS')
# 기울여 집는 윗면 기울기 범위 (°) — 위 끝은 집기 노드 기울여 집기 계산과 같은 motion_math.TILT_DEG(45°). 10/13 PL: tool_tilt_max_deg(30) 키를
# 지움(30 ~ 35° 안전 모드는 충돌 민감도 오설정 탓). 위 끝 값은 석형님 확인
TILT_DEG = (3.0, NODE_TILT_DEG[1])
# 받침 경유 세우기(W131 교시 동작, 10/13): 목표 자세(위 축) → (집기 노드 잡기 이름, 교시 동작 이름). 블록 방향 yaw 는 세운 뒤
# find_blocks 규약(세움 = WIDTH 변 · 옆세움 = LENGTH 변 방향) — 교시대로 긴 축 y 로 눕힌 블록을 세우면 세움 0° · 옆세움 90°
STANDUP = {'LENGTH': ('STANDUP_HIGH', 'high', 0.0), 'WIDTH': ('STANDUP_SIDE', 'side', 90.0)}
# 받침 맞추기(10/14 실기): 맞춘 뒤 다시 잰 블록 가운데가 받침 자리에서 이 안이면 맞은 것(박진용 2 mm — 세우기 손가락 여유 한쪽 약 6 mm),
# 넘으면 다시 맞춘다 — 블록 하나에 최대 CENTER_MAX 번(넘겨도 사람을 부르지 않고 이어 간다 — 일반은 잰 자리에서 집고 세우기는 그대로 세움, 박진용)
CENTER_TOL_M = 0.002
CENTER_MAX = 2

# 실패 뒤 동작. 이유만으로 정하지 않는다 — 실행 단계(phase)와 잡힘 상태(grasped)로 '접촉 전'이 확인될 때만 후보를 다시 쓴다.
# PLAN_FAILED 는 접근 계획뿐 아니라 집은 뒤 들어 올리기 · 운반 · 놓기에서도 나온다. 쥐었거나 상태가 불명확하면 기존 정지 · 복구 절차(RECOVER)로 넘긴다.
PRE_CONTACT = 'PRE_CONTACT'                # 손가락이 블록에 닿기 전 단계(접근 계획 · 이동)에서 실패했다고 확인됨
REPLAN_SAME, WAIT, REOBSERVE, RECOVER, ERROR = 'REPLAN_SAME', 'WAIT', 'REOBSERVE', 'RECOVER', 'ERROR'
ALWAYS_RECOVER = ('TIMEOUT', 'STOPPED', 'CANCELED')                   # 중간에 멈췄다 — 닿았는지 · 쥐었는지 모른다
ALWAYS_ERROR = ('ERROR', 'GRIPPER_NO_RESPONSE', 'NO_FEEDBACK')        # 사람 호출 (그리퍼 응답 · 피드백 없음은 작업 관리자 ERROR, E-74)


def failure_action(reason, phase=None, grasped=None, area_unchanged=None):
    """실패 이유 + 실행 단계 + 잡힘 상태 → 다음 동작.

    phase: PRE_CONTACT 이면 접촉 전 실패가 확인됨, 그 밖(None 포함)은 불명확. grasped: 그리퍼가 블록을 쥐고 있나(False 가 확인돼야 안전).
    area_unchanged: 다른 동작이 공급 영역을 안 바꿨다는 확인(BUSY 만 본다).
    반환: REPLAN_SAME(같은 관측 후보로 다시 계획) · WAIT(기다렸다 같은 요청, 후보 유지) · REOBSERVE(후보 버리고 다시 관측 — 새 집기 전에
    손에 블록이 없음이 확인된 경우만) · RECOVER(기존 정지 · 복구 절차로 — 단순 재관측 뒤 새 집기를 시작하지 않는다) · ERROR(사람 호출).
    """
    if reason == 'LOOKUP_FAILED':
        return REOBSERVE                         # 조회만 실패했다 — 로봇은 움직이지 않았다(횟수 상한은 정책 미정)
    if reason in ALWAYS_ERROR:
        return ERROR
    if reason in ALWAYS_RECOVER:
        return RECOVER
    if grasped is not False:                     # 쥐었거나(True) 모르면(None) 복구 절차
        return RECOVER if reason in ('PLAN_FAILED', 'BUSY', 'GRASP_FAILED', 'SLOT_EMPTY') else ERROR
    if reason == 'PLAN_FAILED':
        return REPLAN_SAME if phase == PRE_CONTACT else RECOVER
    if reason == 'BUSY':
        return WAIT if area_unchanged is True else REOBSERVE
    if reason in ('GRASP_FAILED', 'SLOT_EMPTY'):
        return REOBSERVE                         # 닿았을 수 있어 블록이 밀렸다 — 이전 후보는 버리되 손에는 없음이 확인됨
    return ERROR                                 # 모르는 이유는 조용히 재사용하지 않는다


STEPS = ('approach', 'grasp', 'lift', 'move', 'place', 'retreat')     # PickPlace 피드백 step
CONTACT_UNKNOWN, POST_CONTACT = 'CONTACT_UNKNOWN', 'POST_CONTACT'


def contact_phase(step, reason):
    """PickPlace 피드백의 마지막 step 과 실패 이유 → 접촉 전후(박진용 확인, 10/7).

    approach 안에는 '블록 위에서 그리퍼 열기 → 블록 옆으로 수직 하강'이 들어 있어서 step 만으로 접촉 전이라고 확정하지 않는다:
    approach + PLAN_FAILED = 접촉 전 실패로 확인된 조합(PRE_CONTACT — 전체 동작에서 안 움직였다는 뜻이 아니다) ·
    approach + 그 밖의 실패 = 접촉 여부 모름(CONTACT_UNKNOWN) ·
    grasp 이후(grasp · lift · move · place · retreat) = 접촉 후(POST_CONTACT). 모르는 step · 없으면 CONTACT_UNKNOWN.
    """
    if step == 'approach':
        return PRE_CONTACT if reason == 'PLAN_FAILED' else CONTACT_UNKNOWN
    if step in STEPS:
        return POST_CONTACT
    return CONTACT_UNKNOWN


def grasped_now(state, result_time, now, max_age_s):
    """gripper/state 로 '지금 쥐고 있나'. 믿을 수 없으면 None(모름 — 복구 절차로 간다).

    state = 마지막 gripper_state/1 dict(stamp = 로봇 PC 시계 초 · grasped). stamp · result_time · now 는 **같은 시계(time.time())** 로 잰 값이어야
    한다 — 시간 제한용 단조 시계(time.monotonic)와 섞지 않는다. 믿는 조건(박진용 확인): 값이 PickPlace 결과를 받은 뒤 시각
    (stamp ≥ result_time)이고 now − stamp ≤ max_age_s(1초마다 + 바뀔 때 보내므로 3초). max_age_s 는 부르는 쪽이 명시로 준다(여기서 정하지 않는다).
    """
    try:
        stamp, grasped = state['stamp'], state['grasped']
    except (KeyError, TypeError):
        return None
    ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and abs(v) != float('inf')
             for v in (stamp, result_time, now, max_age_s))
    if not ok or not isinstance(grasped, bool) or max_age_s <= 0:
        return None
    if stamp < result_time or now - stamp > max_age_s or stamp > now:
        return None
    return grasped


class StepTracker:
    """집기 요청마다 PickPlace 피드백의 마지막 step 을 따로 기억한다(ROS 없음).

    begin(request_id) 로 새 요청을 시작하면 이전 요청의 step 은 지워지고, 이전 요청의 늦은 피드백(request_id 가 다름)은 버린다.
    request_id 는 부르는 쪽이 요청마다 새로 만드는 번호(예: 증가하는 정수)다. 모르는 step 도 기록하지 않는다.
    같은 요청 안에서 단계는 STEPS 순서로만 앞으로 간다: 이미 지난 단계나 같은 단계의 늦은 피드백(예: grasp 뒤에 늦게 온 approach)은 버린다 —
    그러지 않으면 접촉 뒤인데 '접촉 전(PRE_CONTACT)'으로 되돌아가 같은 후보로 다시 계획하게 된다.
    결과(PickPlace result)를 실제로 받은 시각도 요청마다 한 번만 적는다(on_result). 결과 없이 끝난 요청(거절 · 시간 초과 · 우리 쪽 취소)은 None 이다.
    """

    def __init__(self):
        """요청 없음으로 시작한다."""
        self._lock = threading.Lock()
        self._current = None
        self._step = None
        self._result_at = None

    def begin(self, request_id):
        """새 요청을 시작한다. 마지막 step 과 결과 수신 시각을 비운다."""
        with self._lock:
            self._current, self._step, self._result_at = request_id, None, None

    def on_feedback(self, request_id, step):
        """피드백을 받았다. 지금 요청의 것이고 알려진 step 이며 마지막 step 보다 **앞으로 가는** 것일 때만 기록한다. 반환: 기록했으면 True."""
        with self._lock:
            if request_id != self._current or request_id is None or step not in STEPS:
                return False
            if self._step is not None and STEPS.index(step) <= STEPS.index(self._step):
                return False                       # 역행 · 중복은 버린다
            self._step = step
            return True

    def on_result(self, request_id, received_at):
        """결과를 받았다. 지금 요청의 첫 결과일 때만 수신 시각(time.time() 초)을 적는다. 반환: 적었으면 True."""
        with self._lock:
            if request_id != self._current or request_id is None or self._result_at is not None:
                return False
            self._result_at = received_at
            return True

    def last_step(self, request_id):
        """그 요청의 마지막 step. 지금 요청이 아니거나 피드백이 없었으면 None."""
        with self._lock:
            return self._step if request_id == self._current and request_id is not None else None

    def result_at(self, request_id):
        """그 요청의 결과 수신 시각(time.time() 초). 지금 요청이 아니거나 결과를 못 받았으면 None."""
        with self._lock:
            return self._result_at if request_id == self._current and request_id is not None else None

    def end(self, request_id):
        """요청이 끝났다(결과 처리 뒤). 더 이상 그 요청의 피드백 · 결과를 받지 않는다."""
        with self._lock:
            if request_id == self._current:
                self._current = self._step = self._result_at = None




def _finite(v):
    import math
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def block_pose(cfg, up, yaw_deg, x_m, y_m, top_z_m):
    """관측한 블록의 중심 자세 (xyz m, 쿼터니언 xyzw). 공급 칸 자세 계산(slot_block_pose)과 같은 규약이다.

    up = 위를 향한 블록 축, yaw_deg = 긴 변이 base x 와 이루는 각(세운 블록은 윗면 25 mm 변 기준, find_blocks 와 같음).
    중심 높이 = 윗면 − 위로 향한 치수의 절반(block_actual_m). 윗면 높이에 카메라 보정 오차가 있으면 그대로 따라온다.
    """
    import math
    yaw = math.radians(yaw_deg)
    if up == 'THICKNESS':
        rot = rpy_matrix(0.0, 0.0, yaw)
    elif up == 'WIDTH':
        rot = rot_z(rpy_matrix(math.pi / 2, 0.0, 0.0), yaw)
    elif up == 'LENGTH':
        rot = rot_z(rpy_matrix(0.0, -math.pi / 2, 0.0), yaw - math.pi / 2)
    else:
        raise ValueError(f'up={up!r}')
    center = (x_m, y_m, top_z_m - half_height(rot, cfg['block_actual_m']))
    return center, quat_from_axes(*(column(rot, k) for k in range(3)))


class ScatterFlow:
    """흩뿌림 집기 한 건의 준비와 재시도 판단. 입력 dict 는 바꾸지 않고 복사해서 쓴다.

    입력: cfg(robot.yaml dict), picker(BlockPicker), supply_mode · open_width_m(명시 주입 — 둘 다 정해져야 실행 가능한 요청을 만든다).
    바깥 영향: 없음(계산만). 로봇 · 서비스를 부르지 않는다.
    """

    def __init__(self, cfg, picker, supply_mode=None, open_width_m=None):
        """설정을 받는다. open_width_m 은 None(미정) 또는 0 이상 유한한 수(m)."""
        if not isinstance(picker, BlockPicker):
            raise ValueError('picker 가 BlockPicker 가 아니다')
        if open_width_m is not None and not (isinstance(open_width_m, (int, float)) and not isinstance(open_width_m, bool)
                                              and 0 <= open_width_m < float('inf')):
            raise ValueError(f'open_width_m 이 이상하다: {open_width_m!r}')
        self.cfg, self.picker = cfg, picker
        self.supply_mode, self.open_width_m = supply_mode, open_width_m
        self._token = 0
        self._lock = threading.Lock()
        self.candidate = None                 # 지금 유효한 집기 요청(commit 으로만 정해지고, 재계획 때만 다시 쓴다)

    @property
    def configured(self):
        """흩뿌림 모드가 명시됐고 열림 폭이 정해졌나. 아니면 실행 가능한 요청을 만들지 않는다."""
        return self.supply_mode == SUPPLY_SCATTER and self.open_width_m is not None

    # ---------- 늦은 답 막기 ----------
    def begin(self, epoch):
        """관측 · 조회를 시작한다. 이전 요청은 무효가 되고, 같은 epoch(상태 변화 표)와 함께 쓸 표를 돌려준다. 후보도 비운다."""
        with self._lock:
            self._token += 1
            self.candidate = None
            return (self._token, epoch)

    def commit(self, ticket, state_fn, result):
        """prepare() 결과를 적용한다. 요청 번호 · 상태 · 정지 · 종료 재확인과 후보 적용을 한 구간에서 한다. 반환: 적용했으면 True.

        state_fn() → (epoch, halted) 를 이 구간 안에서 한 번 읽는다. 부르는 쪽(TaskManager)은 **자기 잠금을 잡은 채** 부른다 —
        그래야 확인 직후 정지 신호가 끼어들지 못한다(정지는 같은 잠금을 얻어야 반영된다). 계산(prepare)은 잠금 밖에서 한다.
        PICK 이면 후보를 정하고, 그 밖의 결과면 후보를 비운다. 늦은 답이면 아무것도 안 바꾼다.
        """
        with self._lock:
            epoch, halted = state_fn()
            if halted or ticket != (self._token, epoch):
                return False
            if result.get('status') in ('PICK', 'SPARE', 'SPARE_TIP', 'SPARE_CENTER'):
                self.candidate = {'request': copy.deepcopy(result['request']), 'candidate': copy.deepcopy(result['candidate']),
                                  'gap_mm': result['gap_mm']}
            else:
                self.candidate = None
            return True

    def invalidate(self):
        """후보를 버린다(정지 · 종료 · 상태 변화 때 TaskManager 가 부른다)."""
        with self._lock:
            self._token += 1
            self.candidate = None

    # ---------- 후보 고르기와 요청 준비 ----------
    def prepare(self, target, response):
        """목표 블록 + find_blocks 응답 → 집기 요청 준비. 반환 dict 의 status:

        (후보는 이 함수가 정하지 않는다 — 계산만. 적용은 commit.)
        PICK          request = PickPlace 목표 칸(part_id · supply_slot '' · grasp · pick_pose · place_pose · open_width_m · obstacles)과 candidate(복사본)
                      obstacles = 같은 응답에서 고른 블록 하나(인덱스)만 뺀 나머지 전부의 중심 자세 [(xyz m, 쿼터니언 xyzw)](under · top 포함, 자르기 없음).
                      고른 블록만 있었으면 빈 목록 — 정상이다(다른 블록 없음, 공급 칸 가정 상자로 안 돌아감).
                      기울어진(tilted) 블록도 장애물에 넣는다 — 지금(핵심 단계)은 집기 요청을 막지 않고 IRD 4.2 그대로 '윗면 = 가장 높은 점인 반듯한 상자'로
                      싣는다(E-68, 10/8 PL). 챌린지 ④(W135)에서 민범진의 기울기 칸으로 정확한 기울기 상자로 바꾼다. 깔린 블록(under)도 장애물이다.
                      기울어진 블록을 **고르지 않는** 것은 BlockPicker 가 한다(E-43): 그런 블록만 남으면 아래 NO_MATCH.
        BLOCKED       요청을 만들지 않는다(reason): INVALID_OBSTACLE = 고르지 않은 블록 중 형식이 틀린 것이 있다(조용히 빼지 않는다 — 상자를 모른다).
                      사람 안내는 guide 'CHECK_BLOCKS'.
        NOT_CONFIGURED 모드 · 열림 폭이 아직 정해지지 않음 — 요청을 만들지 않는다
        EMPTY         공급에 블록이 없다 → 채우기 안내(guide 'REFILL')
        NO_MATCH      블록은 있는데 맞는 후보가 없다 → 제외 이유(counts · message)를 안내(guide 'CHECK_BLOCKS')
        LOOKUP_FAILED 조회 실패 · 잘못된 응답 — 공급 부족이 아니다(reason)
        target = {part_id, grasp, place_pose}. 목표 part_id · 놓을 자세 · 잡기는 그대로 유지하고 복사해서 쓴다.
        """
        if not self.configured:
            return {'status': 'NOT_CONFIGURED'}
        if not isinstance(response, dict) or response.get('ok') is not True or not isinstance(response.get('blocks'), list):
            return {'status': 'LOOKUP_FAILED', 'reason': 'BAD_RESPONSE'}
        up, axis = grasp_target(target['grasp'], self.cfg)
        picked = self.picker.pick(response['blocks'], up, axis)
        if picked['status'] == 'NONE':
            if picked['reason'] == 'EMPTY':
                return {'status': 'EMPTY', 'guide': 'REFILL'}
            if picked['counts']['invalid'] == len(response['blocks']):
                return {'status': 'LOOKUP_FAILED', 'reason': 'ALL_INVALID'}      # 쓸 수 있는 블록이 하나도 없는 응답 — 조회 오류
            return {'status': 'NO_MATCH', 'guide': 'CHECK_BLOCKS', 'counts': dict(picked['counts']),
                    'message': self._why(picked['counts'])}
        c = picked['block']
        obstacles = self._obstacles(response['blocks'], picked['index'])
        if isinstance(obstacles, dict):
            return obstacles
        center, quat = block_pose(self.cfg, c['up'], c['yaw_deg'], c['x_m'], c['y_m'], c['top_z_m'])
        request = {'part_id': target['part_id'], 'supply_slot': '', 'grasp': target['grasp'],
                   'pick_pose': (center, quat), 'place_pose': copy.deepcopy(target['place_pose']),
                   'open_width_m': self.open_width_m, 'obstacles': obstacles}
        return {'status': 'PICK', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(c), 'gap_mm': picked['gap_mm']}

    # ---------- 받침 경유(10/13 박진용 — one_pick 실기 레시피 4 · 1 끝까지, PL 10시 지시로 정식 흐름에) ----------
    @property
    def via_spare(self):
        """robot.yaml scatter.via_spare 가 참이면 흩뿌림 집기를 받침 경유로 한다(아니면 지금 흐름 그대로)."""
        return bool((self.cfg.get('scatter') or {}).get('via_spare'))

    def spare_spot(self):
        """받침 자리 = robot.yaml spare_spots 첫 자리 {x_m, y_m, z_m(받침 윗면), yaw_deg}. 형식이 틀리면 ValueError."""
        spots = self._spare_spots(self.cfg.get('spare_spots', []))
        if not spots or not _finite(spots[0].get('z_m', self.cfg['table_z_m'])):
            raise ValueError('robot.yaml spare_spots 첫 자리가 없거나 z_m 이 숫자가 아니다')
        return spots[0]

    def prepare_spare(self, target, response, failed=()):
        """받침 경유 1단계 — 공급에서 집기 쉬운 블록을 BlockPicker.rank 순서로 골라 받침에 눕혀 놓는 요청을 준비한다(계산만).

        입력: target {part_id, grasp, place_pose}(레시피 목표 — 2단계에서 씀), response = find_blocks 응답(height_map 칸이 있으면 손가락 자리를
        깊이로 미리 거른다), failed = 이번 블록에서 안 된 시도 [(x_m, y_m, grasp, dyaw_deg, slide_m)](같은 후보를 다시 고르지 않게).
        잡기는 집기 쉬운 것(틀기 · 옮기기 · 기울여 집기 TILT 포함) — 받침에서 다시 재서 레시피 잡기로 곧게 다시 잡으므로 여기 잡기는 조립과 상관없다.
        옆으로 선 · 선 블록(EDGE_* · STAND_*)은 그 자세 그대로 받침에 세워 놓는다(one_pick — 받침에서 다시 재고 잡기 TIP 으로 눕힌다) — 레시피와 같은 자세면 그대로 집는다.
        세움 · 옆세움 레시피는 세우기 교시(robot.yaml standup_taught)가 있을 때만 받침으로 옮긴다(없으면 STAND_NOT_READY).
        옆으로 섬은 긴 축이 받침 방향(spot yaw_deg), 섬은 넘어지는 쪽(yaw + 90°)이 받침 긴 변(support_long_yaw)이 되게 — 짧은 변으로 넘기면
        넘기기 전 세운 자리가 받침 밖(가운데에서 44.6 mm).
        반환 status: SPARE(request — 받침에 놓는 PickPlace 칸 + 내부 표시 'via_spare': True, attempt = failed 에 넣을 꼴, tag = 고른 이유)
        · EMPTY · NO_MATCH(counts · message) · NOT_CONFIGURED · LOOKUP_FAILED · BLOCKED(값이 틀린 블록이 하나라도 있음 · 받침 자리 설정 틀림). 바깥 영향 없음.
        """
        if not self.configured:
            return {'status': 'NOT_CONFIGURED'}
        if not isinstance(response, dict) or response.get('ok') is not True or not isinstance(response.get('blocks'), list):
            return {'status': 'LOOKUP_FAILED', 'reason': 'BAD_RESPONSE'}
        try:
            spot = self.spare_spot()
        except ValueError as e:
            return {'status': 'BLOCKED', 'reason': 'INVALID_SPARE_SPOTS', 'guide': 'CHECK_BLOCKS', 'message': str(e)}
        blocks = [self._with_tilt(b) for b in response['blocks']]
        if not blocks:
            return {'status': 'EMPTY', 'guide': 'REFILL'}
        bad = next((k for k, b in enumerate(response['blocks']) if not self.picker.is_valid(b)), None)
        if bad is not None:
            # 값이 틀린 블록(up 모름 등)은 손가락 자리 · 장애물 계산에 못 쓴다 — 고르기 전에 막고 다시 찍는다(_obstacles 와 같은 규칙).
            # 예전엔 순위 계산(rank)이 이웃 블록 상자를 만들다 예외로 멈췄다(#152 리뷰 6, 재현됨)
            return {'status': 'BLOCKED', 'reason': 'INVALID_OBSTACLE', 'index': bad, 'guide': 'CHECK_BLOCKS',
                    'message': f'블록 {bad}번의 값이 잘못돼 집기를 시작하지 않는다'}
        if not self.can_stand(grasp_target(target['grasp'], self.cfg)[0]):
            return {'status': 'STAND_NOT_READY'}           # 받침에서 세우는 동작(robot.yaml standup_taught)이 없다 — 공급에서 옮기기 전에 막는다
        pts = height_points(response.get('height_map'))
        ranked, cut = self.picker.rank(blocks, list(failed), pts)
        # 깊이를 못 받으면(높이 지도 없음 · 점 0개) 기울여 집기는 뺀다 — 손가락 자리 · 윗면 평면을 깊이로 못 본 채 기울여 내려가지 않는다
        # (one_pick NO_DEPTH. 점 0개를 '받음'으로 보던 것 — 10/14 점검 ② 5)
        depth = pts is not None and len(pts) > 0
        ranked = [r for r in ranked if r[1] in SPARE_GRASPS and (depth or r[1] != 'TILT')]
        if not ranked:
            n = len(blocks)
            return {'status': 'NO_MATCH', 'guide': 'CHECK_BLOCKS', 'counts': {'no_clear': n},
                    'message': f'손가락이 들어갈 자리가 있는 블록이 없음(블록 {n}개 · 깊이로 거른 후보 {cut.get("깊이", 0)}개)'}
        i, grasp, dyaw, slide, tag = ranked[0]
        b = blocks[i]
        obstacles = self._obstacles(response['blocks'], i)
        if isinstance(obstacles, dict):
            return obstacles
        thick = self.cfg['block_actual_m'][2]
        if grasp == 'TILT':
            # 기울여 집기: 윗면 법선 방향으로 두께 절반만 문다(10/12 실기 — 평소 깊이로 대각선으로 들어가 밑 블록을 찍음). 노드는 윗면에서
            # grasp_depth_m 을 무니 집기 자세를 법선 방향으로 그 차이만큼 올리고, 얕게 문 만큼 블록이 손 아래로 더 나와 놓는 자리도 그만큼 올린다
            center, quat, normal = tilt_block_pose(b, thick)
            shift = self.cfg['grasp_depth_m'] - thick / 2
            pick_pose = (tuple(c + n * shift for c, n in zip(center, normal)), quat)
        else:
            shift = 0.0
            pick_pose = self._grasp_pose(b, grasp, dyaw, slide)
        up = 'THICKNESS' if grasp == 'TILT' else b['up']
        yaw = self.support_long_yaw() - 90.0 if up == 'LENGTH' else spot['yaw_deg']
        size = dict(zip(SIZE_KEYS, self.cfg['block_actual_m']))[up]
        sc, sq = block_pose(self.cfg, up, yaw, spot['x_m'], spot['y_m'], spot.get('z_m', self.cfg['table_z_m']) + size)
        request = {'part_id': target['part_id'], 'supply_slot': '', 'grasp': grasp, 'pick_pose': pick_pose,
                   'place_pose': ((sc[0], sc[1], sc[2] + shift), sq), 'open_width_m': self.open_width_m,
                   'obstacles': obstacles, 'via_spare': True}
        return {'status': 'SPARE', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(response['blocks'][i]), 'gap_mm': None,
                'attempt': (b['x_m'], b['y_m'], grasp, dyaw, slide), 'tag': tag}

    def can_stand(self, up):
        """목표 자세 up 을 받침 경유로 만들 수 있나 — 눕힘은 늘, 세움 · 옆세움은 robot.yaml standup_taught(세우기 교시)에 그 동작이 있을 때."""
        if up == 'THICKNESS':
            return True
        taught = self.cfg.get('standup_taught') or {}
        return up in STANDUP and STANDUP[up][1] in taught

    def support_long_yaw(self):
        """받침 긴 변 방향 (°) — robot.yaml spare_base_m 의 x 범위가 y 범위보다 길면 0, 아니면 90(세운 블록을 넘기는 쪽, one_pick 과 같음)."""
        sb = self.cfg.get('spare_base_m') or {'x_m': [0.0, 1.0], 'y_m': [0.0, 0.0]}
        return 0.0 if sb['x_m'][1] - sb['x_m'][0] >= sb['y_m'][1] - sb['y_m'][0] else 90.0

    def reseen_request(self, goal, cand, response):
        """헛잡음 뒤 다시 찍은 find_blocks 응답에서 그 블록(cand 자리 40 mm 안 · 같은 자세 · 확신도 0.5 이상 · 덮이지 않음 — one_pick see_block)을 찾아
        같은 잡기 · 각도 틀기 · 손가락 옮김(goal['attempt'])으로 pick_pose 만 고친 요청. 못 찾으면 None. 바깥 영향 없음."""
        if not isinstance(response, dict) or response.get('ok') is not True or not isinstance(response.get('blocks'), list) or not cand:
            return None
        near = [b for b in response['blocks'] if b.get('up') == cand.get('up') and b.get('overlap') != 'under'
                and (b.get('confidence') or 0.0) >= 0.5 and BlockPicker.is_valid(b)
                and math.hypot(b['x_m'] - cand['x_m'], b['y_m'] - cand['y_m']) < 0.04]
        if not near:
            return None
        b = min(near, key=lambda o: math.hypot(o['x_m'] - cand['x_m'], o['y_m'] - cand['y_m']))
        _x, _y, grasp, dyaw, slide = goal['attempt']
        req = {k: v for k, v in goal.items() if k not in ('status', 'regrasp')}
        req['pick_pose'], req['seen'] = self._grasp_pose(b, grasp, dyaw, slide), copy.deepcopy(b)
        obstacles = self._obstacles(response['blocks'], response['blocks'].index(b))
        if isinstance(obstacles, dict):
            return None
        req['obstacles'] = obstacles
        return copy.deepcopy(req)

    def prepare_from_spare(self, target, response, centers=0):
        """받침 경유 2단계 — 받침 위 블록(find_blocks area 'spare' — 깊이로 잰 블록 하나)을 보고 다음 요청을 준비한다(계산만, one_pick 과 같은 순서).
        받침은 공급 영역 밖 블록 하나뿐인 자리라 장애물 없음.

        ⓪ 옆으로 선 · 선 블록이 레시피와 같은 자세면(세우기 뒤) PICK — 눕히지 않고 레시피 잡기로 바로 집어 조립한다.
        ① 그 밖의 옆으로 선 · 선 블록이면 SPARE_TIP — 잡기 TIP 으로 받침 가운데에 눕힌다(누울 자리 yaw: 옆으로 섬 = spot yaw_deg, 섬 = 받침 긴 변).
           눕힌 뒤 받침을 다시 잰다. ② 눕힌 블록이면 받침 맞추기(SPARE_CENTER) — 잰 중심 · 방향에서 레시피 잡기의 반대 폭(<반대 잡기>)으로 집어
           받침 가운데(spot · yaw_deg)에 옮겨 놓는다(10/14 — 받침에 밀려 · 틀어져 놓인 것을 없앰). 맞춘 뒤 작업 관리자가 받침을 다시 재서
           centers(이 블록에 맞추기를 한 수)를 하나 올려 다시 부른다 — 받침 자리에서 CENTER_TOL_M 안이면 다음으로, 넘으면 CENTER_MAX 번까지 다시 맞춘다.
           일반(FLAT_SHORT · FLAT_LONG): 맞은 뒤(또는 CENTER_MAX 를 넘기면) 잰 자리에서 레시피 잡기로 집는다(PICK). 맞추기가 움직이기 전에 길이
           안 나오면 잰 값 그대로 집는다(request['skip']).
        ②' 세움 · 옆세움 레시피(STAND_* · EDGE_*): 같은 방법으로 맞춘 뒤 맞았으면 세우기 교시 동작(SPARE_STAND — STANDUP_HIGH · STANDUP_SIDE,
           10/13 W131)으로 세운다 — 세운 뒤 다시 재면 ⓪ 으로 집는다(_stand_request). CENTER_MAX 번에도 안 맞으면 사람을 부르지 않고 그대로
           세운다(박진용). 교시가 없으면 STAND_NOT_READY(사람).
           centers > 0 인 결과에는 다시 잰 어긋남 offset_mm(받침 자리에서 블록 가운데까지 수평 거리)를 붙인다(작업 관리자가 기록).
        ③ 그 밖은 PICK — 잰 중심 · 방향에서 레시피 잡기로 곧게 다시 집어 조립 자리(target place_pose)에 놓는다.
        반환 status: SPARE_TIP · SPARE_CENTER · SPARE_STAND · PICK(request · candidate) · SPARE_EMPTY(받침에 블록이 안 보임) · STAND_NOT_READY
        · SPARE_BLOCKED(받침 둘레 손가락 자리에 높은 점 — message)
        · SPARE_NOT_FLAT(자세를 모름 — block) · NOT_CONFIGURED · LOOKUP_FAILED. 바깥 영향 없음.
        """
        if not self.configured:
            return {'status': 'NOT_CONFIGURED'}
        if not isinstance(response, dict) or response.get('ok') is not True or not isinstance(response.get('blocks'), list):
            return {'status': 'LOOKUP_FAILED', 'reason': 'BAD_RESPONSE'}
        if not response['blocks']:
            return {'status': 'SPARE_EMPTY'}
        b = response['blocks'][0]
        try:
            spot = self.spare_spot()
        except ValueError as e:
            return {'status': 'LOOKUP_FAILED', 'reason': str(e)}
        size = dict(zip(SIZE_KEYS, self.cfg['block_actual_m']))
        base = b.get('base_z_m', spot.get('z_m', self.cfg['table_z_m']))
        thick = size['THICKNESS']
        want = grasp_target(target['grasp'], self.cfg)[0]
        if b.get('up') in ('WIDTH', 'LENGTH') and b['up'] == want:
            # 받침에 레시피 자세로 선 블록(세우기 뒤) — 눕히지 않고 레시피 잡기로 바로 집어 조립(높이는 받침 윗면 + 그 자세 높이)
            pick_pose = block_pose(self.cfg, b['up'], b['yaw_deg'], b['x_m'], b['y_m'], base + size[b['up']])
            request = {'part_id': target['part_id'], 'supply_slot': '', 'grasp': target['grasp'], 'pick_pose': pick_pose,
                       'place_pose': copy.deepcopy(target['place_pose']), 'open_width_m': self.open_width_m, 'obstacles': []}
            return {'status': 'PICK', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(b), 'gap_mm': None}
        if b.get('up') in ('WIDTH', 'LENGTH'):
            # 받침에 그 자세로 세워 둔 블록 — 높이는 받침 윗면 + 그 자세 높이(잰 높이 흔들림을 버림)
            pick_pose = block_pose(self.cfg, b['up'], b['yaw_deg'], b['x_m'], b['y_m'], base + size[b['up']])
            yaw = self.support_long_yaw() if b['up'] == 'LENGTH' else spot['yaw_deg']
            land = block_pose(self.cfg, 'THICKNESS', yaw, spot['x_m'], spot['y_m'], base + thick)
            request = {'part_id': target['part_id'], 'supply_slot': '', 'grasp': 'TIP', 'pick_pose': pick_pose, 'place_pose': land,
                       'open_width_m': self.open_width_m, 'obstacles': [], 'spare_tip': True}
            return {'status': 'SPARE_TIP', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(b), 'gap_mm': None}
        if b.get('up') != 'THICKNESS':
            return {'status': 'SPARE_NOT_FLAT', 'block': copy.deepcopy(b)}
        off = math.hypot(b['x_m'] - spot['x_m'], b['y_m'] - spot['y_m'])
        recenter = centers == 0 or (off > CENTER_TOL_M and centers < CENTER_MAX)
        if want != 'THICKNESS':
            return self._with_offset(self._stand_request(target, b, spot, base, want, recenter, off), centers, off)
        # 윗면 = 받침 윗면 + 두께(잰 높이는 1 ~ 2 mm 흔들려 그대로 물면 손 안에서 매달려 놓을 때 받침을 누른다 — 10/11 실기)
        pick_pose = block_pose(self.cfg, 'THICKNESS', b['yaw_deg'], b['x_m'], b['y_m'], base + thick)
        request = {'part_id': target['part_id'], 'supply_slot': '', 'grasp': target['grasp'], 'pick_pose': pick_pose,
                   'place_pose': copy.deepcopy(target['place_pose']), 'open_width_m': self.open_width_m, 'obstacles': []}
        # 받침 둘레 손가락 자리를 깊이로 본다(응답에 height_map 이 있을 때 — one_pick p1_to_recipe). 막히면 사람이 비운다
        pts = height_points(response.get('height_map'))
        if pts is not None and len(pts):
            from d2_task.pick_rank import PickRanker
            ranker = PickRanker(self.cfg)
            fingers, tip_z = ranker.candidate_fingers(dict(b, top_z_m=base + thick), target['grasp'], 0, 0.0)
            hit = ranker.fingers_blocked(fingers, tip_z, pts)
            if hit:
                return {'status': 'SPARE_BLOCKED', 'message': ', '.join(f'{k}번 손가락 {f * 100:.0f} %' for k, f in hit)}
        if recenter and target['grasp'] in ('FLAT_SHORT', 'FLAT_LONG'):
            # 받침 맞추기(10/14): 잰 자리에서 반대 폭으로 집어 받침 가운데에 옮겨 놓는다 — 그 자리에서 물기만 하면 밀려 놓인 블록이 그대로 남았다
            other = 'FLAT_LONG' if target['grasp'] == 'FLAT_SHORT' else 'FLAT_SHORT'
            spot_pose = block_pose(self.cfg, 'THICKNESS', spot['yaw_deg'], spot['x_m'], spot['y_m'], base + thick)
            center = dict(request, grasp=other, place_pose=spot_pose, spare_center=True, skip=copy.deepcopy(request))
            return self._with_offset({'status': 'SPARE_CENTER', 'request': copy.deepcopy(center), 'candidate': copy.deepcopy(b), 'gap_mm': None},
                                     centers, off)
        return self._with_offset({'status': 'PICK', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(b), 'gap_mm': None}, centers, off)

    @staticmethod
    def _with_offset(result, centers, off):
        """맞추기를 한 뒤(centers > 0) 결과에 다시 잰 어긋남 offset_mm 를 붙인다(작업 관리자 기록용). 결과 dict 를 그대로 돌려준다."""
        if centers > 0:
            result['offset_mm'] = round(off * 1000.0, 1)
        return result

    def _stand_request(self, target, b, spot, base, up, recenter, off):
        """받침 위 눕힌 블록 b(잰 값)를 세우는 요청(세움 · 옆세움 레시피, W131 교시 동작) — recenter 면 받침 가운데에 옮겨 놓고(SPARE_CENTER),
        아니면 세운다(SPARE_STAND — 맞았거나 CENTER_MAX 번을 다 했음). off = 받침 자리에서 잰 가운데까지 m(기록용).

        교시 동작은 받침 자리 한 곳(spare_spots 첫 자리 · yaw_deg)의 좌표라 블록이 꼭 그 자리에 있어야 한다. 맞추기는 잰 중심 · 방향에서
        세우기 잡기의 반대 폭(<반대 잡기>)으로 집어 받침 가운데에 놓는다(10/14 — 예전엔 받침 가운데에서 물기만 해서 11 mm 밀린 블록을
        손가락이 찍었다, 10/13 실기). 세우기 요청은 그 자리의 눕힌 블록 → 세운 블록(위 축 up)이고, 끝나면 다음 SELECT 가 받침을 다시 재서
        레시피 잡기로 집는다(spare_stand). 맞추기를 건너뛸 길(skip)은 없다 — 가운데가 아니면 못 세운다.
        반환: SPARE_CENTER · SPARE_STAND(request) · STAND_NOT_READY(교시 없음). 바깥 영향 없음.
        """
        if not self.can_stand(up):
            return {'status': 'STAND_NOT_READY'}
        grasp, case, yaw = STANDUP[up]
        size = dict(zip(SIZE_KEYS, self.cfg['block_actual_m']))
        lying = block_pose(self.cfg, 'THICKNESS', spot['yaw_deg'], spot['x_m'], spot['y_m'], base + size['THICKNESS'])
        seen = block_pose(self.cfg, 'THICKNESS', b['yaw_deg'], b['x_m'], b['y_m'], base + size['THICKNESS'])
        stood = block_pose(self.cfg, up, yaw, spot['x_m'], spot['y_m'], base + size[up])
        stand = {'part_id': target['part_id'], 'supply_slot': '', 'grasp': grasp, 'pick_pose': lying, 'place_pose': stood,
                 'open_width_m': self.open_width_m, 'obstacles': [], 'spare_stand': True}
        if not recenter:
            return {'status': 'SPARE_STAND', 'request': copy.deepcopy(stand), 'candidate': copy.deepcopy(b), 'gap_mm': None}
        under = self.cfg['standup_taught'][case]['grasp']
        other = 'FLAT_LONG' if under == 'FLAT_SHORT' else 'FLAT_SHORT'
        center = dict(stand, grasp=other, pick_pose=seen, place_pose=lying, spare_center=True)
        center.pop('spare_stand')
        return {'status': 'SPARE_CENTER', 'request': copy.deepcopy(center), 'candidate': None, 'gap_mm': None}

    def _grasp_pose(self, b, grasp, dyaw, slide):
        """블록 b 를 이 잡기 · 그리퍼 각도 틀기(dyaw °) · 손가락 옮김(slide m)으로 집는 pick_pose — 튼 만큼 블록 방향을 그렇게 보고 집는다.
        옮김은 눕힌 블록만(좁은 폭은 길이 방향, 넓은 폭은 폭 방향). 윗면은 층 높이(PickRanker.top_of — 카메라 높이 흔들림을 버림)."""
        x, y = b['x_m'], b['y_m']
        if slide:
            axis = math.radians(b['yaw_deg'] if grasp == 'FLAT_SHORT' else b['yaw_deg'] + 90)
            x, y = x + slide * math.cos(axis), y + slide * math.sin(axis)
        return block_pose(self.cfg, b['up'], b['yaw_deg'] + dyaw, x, y, self.picker.top_of(b))

    def _with_tilt(self, b):
        """기울어진 눕힌 블록에 기울여 집기 후보용 tilt_tip_z_m(두께 절반 무는 손끝 높이)을 붙인 복사본. find_blocks 가 준 center_top_z_m · normal 이
        없거나 기울기가 TILT_DEG 밖이면 붙이지 않는다(그 블록은 후보에서 빠진다 — BlockPicker.rank 규칙). 반듯한 블록은 그대로."""
        b = dict(b)
        if not (b.get('tilted') is True and b.get('up') == 'THICKNESS'):
            return b
        n, top = b.get('normal'), b.get('center_top_z_m')
        if not (isinstance(n, (list, tuple)) and len(n) == 3 and all(_finite(v) for v in n) and _finite(top)):
            b.pop('center_top_z_m', None)
            return b
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, n[2]))))
        if not TILT_DEG[0] <= tilt <= TILT_DEG[1]:
            b.pop('center_top_z_m', None)
            return b
        b['tilt_tip_z_m'] = top - self.cfg['block_actual_m'][2] / 2 * n[2]
        return b

    @staticmethod
    def _spare_spots(spots):
        """robot.yaml spare_spots → 검증한 목록(값은 그대로). 형식이 틀리면 ValueError(조용히 빼지 않는다) — 세울 때만 부른다."""
        if not isinstance(spots, list):
            raise ValueError('robot.yaml spare_spots 가 목록이 아니다')
        for k, s in enumerate(spots, 1):
            if not isinstance(s, dict) or not all(_finite(s.get(key)) for key in ('x_m', 'y_m', 'yaw_deg')):
                raise ValueError(f'robot.yaml spare_spots {k}번이 {{x_m, y_m, yaw_deg}} 숫자가 아니다')
        return [dict(s) for s in spots]

    def _obstacles(self, blocks, picked_index):
        """같은 응답에서 고른 인덱스 하나만 빼고 나머지를 장애물 자세로 바꾼다. 반환: 자세 목록, 또는 요청을 막는 BLOCKED dict.

        나머지 중 형식이 틀린 블록이 있으면 일부만 보내지 않고 요청을 막는다(조용히 빼면 그 자리를 비었다고 보고 지나간다).
        tilted · under(덮임) · top(겹침 중 위)도 모두 상자를 만든다(E-68). tilted 는 지금 '가장 높은 점인 반듯한 상자'(top_z_m 기준), 챌린지 ④ 때 기울기 상자로 바꾼다.
        under 의 top_z_m 이 그 블록 자신의 윗면인지는 비전(민범진) 확인 전이다.
        """
        others = [(i, b) for i, b in enumerate(blocks) if i != picked_index]
        for i, b in others:
            if not self.picker.is_valid(b):
                return {'status': 'BLOCKED', 'reason': 'INVALID_OBSTACLE', 'index': i, 'guide': 'CHECK_BLOCKS',
                        'message': f'장애물 블록 {i}번의 값이 잘못돼 집기를 시작하지 않는다'}
        return [block_pose(self.cfg, b['up'], b['yaw_deg'], b['x_m'], b['y_m'], b['top_z_m']) for _, b in others]

    @staticmethod
    def _why(counts):
        """제외 이유 개수 → 사람이 읽을 안내 글자(0 인 것은 뺀다)."""
        names = {'under': '다른 블록에 덮임', 'tilted': '기울어짐', 'other_up': '필요한 자세가 아님',
                 'no_clear': '손가락 틈 부족', 'narrow': '틈이 기준보다 좁음', 'invalid': '잘못된 값'}
        parts = [f'{names[k]} {n}개' for k, n in counts.items() if n and k in names]
        return '맞는 블록이 없다: ' + (', '.join(parts) if parts else '이유 없음')

    # ---------- 실패 뒤 ----------
    def on_failure(self, reason, phase=None, grasped=None, area_unchanged=None):
        """집기 실패(또는 조회 실패) 뒤 다음 동작(failure_action). REPLAN_SAME · WAIT 이면 지금 후보를 유지하고, 그 밖이면 후보를 버린다."""
        action = failure_action(reason, phase, grasped, area_unchanged)
        if action not in (REPLAN_SAME, WAIT):
            with self._lock:
                self.candidate = None
        return action

    def reusable_request(self):
        """같은 관측으로 다시 계획할 때 쓸 요청(복사본). 후보가 버려졌으면 None."""
        with self._lock:
            return None if self.candidate is None else copy.deepcopy(self.candidate['request'])

    def after_wait_start(self):
        """흩뿌림 대기(WAIT_SUPPLY)에서 [계속](start)을 받았을 때: 다시 관측하고 조회한다. 공급 칸은 초기화하지 않는다."""
        self.invalidate()
        return {'reobserve': True, 'reset_supply_slots': False}


def height_points(hm):
    """find_blocks height_map 칸 {x0_m, y0_m, res_m, nx, ny, z_m(행 우선, 빈 칸 null)} → 칸 가운데 점 [(x, y, z)] m. 없거나 형식이 틀리면 None
    (그러면 BlockPicker.rank 가 깊이로 미리 거르지 않는다 — 노드가 내려가기 전에 다시 보지 않으므로 응답에 이 칸을 꼭 요청한다)."""
    try:
        import numpy as np
        x0, y0, res, nx, ny = float(hm['x0_m']), float(hm['y0_m']), float(hm['res_m']), int(hm['nx']), int(hm['ny'])
        z = np.array([np.nan if v is None else float(v) for v in hm['z_m']], dtype=float)
        if z.size != nx * ny or res <= 0:
            return None
    except (KeyError, TypeError, ValueError):
        return None
    iy, ix = np.divmod(np.arange(nx * ny), nx)
    pts = np.stack([x0 + (ix + 0.5) * res, y0 + (iy + 0.5) * res, z], axis=1)
    return pts[~np.isnan(pts[:, 2])]


def tilt_block_pose(b, thick):
    """기울어진 눕힌 블록 b(find_blocks center_top_z_m · normal · yaw_deg)의 중심 자세 — 회전 열 = 길이 · 폭 · 윗면 법선(집기 노드 TILT 가
    두께 축을 윗면 법선으로 본다). 반환: (중심 xyz m, 쿼터니언 xyzw, 법선)."""
    n = [float(v) for v in b['normal']]
    yaw = math.radians(b['yaw_deg'])
    u0 = (math.cos(yaw), math.sin(yaw), 0.0)
    d = sum(a * c for a, c in zip(u0, n))
    u = [a - d * c for a, c in zip(u0, n)]
    norm = math.sqrt(sum(a * a for a in u))
    u = [a / norm for a in u]
    w = [n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0]]
    top = (b['x_m'], b['y_m'], b['center_top_z_m'])
    center = tuple(t - c * thick / 2 for t, c in zip(top, n))
    return center, quat_from_axes(u, w, n), n
