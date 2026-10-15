# -*- coding: utf-8 -*-
"""손목 그림 집기 순위(state/2 pick, 10/14 22시 PL) 시험 — ROS 없이 ScatterFlow · TaskManager 를 가짜 IO 로 돌린다.

공급 find_blocks 응답(image_seq)에서 PickRanker 순서로 고른 결과(SPARE)가 pick {image_seq, index, ranks, reason} 을 싣고,
작업 관리자는 다음 공급 결과가 올 때까지 모든 state/2 에 그것을 싣다가(받침 위 재기는 바꾸지 않음) 조립 끝 · 취소 · IDLE · 새 출발이면 키를 뺀다.
"""
from d2_task.scatter_pick import height_points
from test_block_picker import blk
from test_scatter_connect import start
from test_task_manager import IDS, SAFE_OK, SAFE_STOP, drive
from test_via_spare import SPARE_CFG, T, W, make

FLOOR_HM = {'x0_m': 0.0, 'y0_m': 0.0, 'res_m': 0.002, 'nx': 1, 'ny': 1, 'z_m': [SPARE_CFG['table_z_m']]}   # 깊이는 받았고 막는 점은 없음
TARGET = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.01), (0, 0, 0, 1))}


def scene(seq=7):
    """우선순위가 다른 블록 넷(서로 멀어 손가락 자리가 안 겹침): 1번 1층 눕힘 · 2번 1층 옆으로 섬 · 3번 덮임(후보 아님) · 4번 2층 눕힘.
    집기 순서 = 4(높은 층 눕힘) → 1(1층 눕힘) → 2(선 블록). seq 가 None 이면 image_seq 칸이 없는 옛 · 가짜 비전 응답."""
    t = SPARE_CFG['table_z_m']
    blocks = [blk(x=0.30, y=0.10, top=t + T, length=60, width=60),
              blk(x=0.30, y=0.30, top=t + W, up='WIDTH', length=60, thickness=60),
              blk(x=0.50, y=0.10, top=t + T, overlap='under', length=60, width=60),
              blk(x=0.50, y=0.30, top=t + 2 * T, length=60, width=60)]
    r = {'ok': True, 'blocks': blocks, 'height_map': FLOOR_HM}
    if seq is not None:
        r['image_seq'] = seq
    return r


def supply_finds(io):
    """지금까지 부른 공급 find_blocks 수(높이 지도 요청 = 공급, area spare = 받침)."""
    return sum(1 for o in io.opts if o.get('height_map'))


# ---------------- ScatterFlow.prepare_spare 결과 ----------------
def test_고른_결과의_pick_은_PickRanker_순서와_같다():
    m, _ = make()
    flow = m._scatter
    resp = scene(7)
    r = flow.prepare_spare(TARGET, resp)
    rows, _ = flow.picker.rank([flow._with_tilt(b) for b in resp['blocks']], [], height_points(resp['height_map']))
    assert r['status'] == 'SPARE'
    assert r['pick'] == {'image_seq': 7, 'index': 4, 'ranks': [4, 1, 2], 'reason': r['tag']}
    assert r['pick']['ranks'] == list(dict.fromkeys(i + 1 for i, *_ in rows))           # 블록마다 처음 나온 자리 한 번 · 덮인 3번 없음
    assert r['pick']['index'] == rows[0][0] + 1 and r['pick']['reason'] == rows[0][4]
    assert r['candidate'] == resp['blocks'][3]                                          # index 는 응답 blocks 의 1부터 번호
    assert r['tag'].startswith('눕힌 블록 · 높이 2층 · 눕힘 곧게 · 열림 0 · ')


def test_image_seq_가_없거나_정수가_아니면_null_로_싣는다():
    m, _ = make()
    assert m._scatter.prepare_spare(TARGET, scene(None))['pick']['image_seq'] is None
    assert m._scatter.prepare_spare(TARGET, dict(scene(), image_seq=True))['pick']['image_seq'] is None
    assert m._scatter.prepare_spare(TARGET, dict(scene(), image_seq='7'))['pick']['image_seq'] is None


def test_못_고른_결과에는_pick_이_없다():
    m, _ = make()
    under = {'ok': True, 'blocks': [scene()['blocks'][2]], 'image_seq': 3}
    r = m._scatter.prepare_spare(TARGET, under)
    assert r['status'] == 'NO_MATCH' and 'pick' not in r


# ---------------- 작업 관리자 state/2 ----------------
def test_공급에서_고른_순위가_다음_공급_결과까지_모든_state_에_실리고_조립이_끝나면_빠진다():
    m, io = make()
    io.find_script = [(True, '', scene(7)), (True, '', scene(8))]                # 세 번째부터는 image_seq 없는 가짜 응답
    start(m)
    assert io.states and all('pick' not in s for s in io.states)                  # 고르기 전(READY · CHECK)에는 키가 없다
    drive(m, 'PICK_PLACE')
    first = len(io.states) - 1
    assert io.states[first]['pick'] == {'image_seq': 7, 'index': 4, 'ranks': [4, 1, 2], 'reason': io.states[first]['pick']['reason']}
    assert io.states[first]['pick']['reason'].startswith('눕힌 블록 · 높이 2층 · ')
    before = len(io.states)
    for _ in range(200):                                                          # 두 번째 공급 찍기 전까지 한 단계씩
        before = len(io.states)
        m.run_once()
        if supply_finds(io) == 2:
            break
    mid = io.states[first:before]
    assert all(s['pick']['image_seq'] == 7 for s in mid)                         # 받침에 옮기기 · 받침 위 재기 · 맞추기 · 조립 · 확인 내내 그대로
    assert {'PICK_PLACE', 'SELECT', 'VERIFY'} <= {s['state'] for s in mid}
    assert any('받침' in s['message'] for s in mid if s['state'] == 'PICK_PLACE' and s is not mid[0])
    assert io.states[-1]['state'] == 'PICK_PLACE' and io.states[-1]['pick']['image_seq'] == 8      # 다음 공급 결과가 바꾼다
    for _ in range(200):
        if supply_finds(io) == 3:
            break
        m.run_once()
    assert io.states[-1]['pick']['image_seq'] is None and io.states[-1]['pick']['index'] >= 1     # image_seq 없는 응답도 pick 은 낸다
    drive(m, 'DONE')
    assert io.states[-1]['state'] == 'DONE' and 'pick' not in io.states[-1]
    n = len(io.states)
    start(m)                                                                     # 새 설계 고르기 · 새 출발 — 앞 조립 순위를 싣지 않는다
    assert all('pick' not in s for s in io.states[n:])


def test_같은_관측의_다음_후보로_가면_같은_그림의_새_순위를_싣는다():
    m, io = make()
    io.find_script = [(True, '', scene(7))]
    io.pick_script = [(False, 'PLAN_FAILED')]                                     # 첫 후보: 움직이기 전 길 거절 → 다시 찍지 않고 같은 관측의 다음 후보
    start(m)
    drive(m, 'PICK_PLACE')
    p1 = io.states[-1]['pick']
    m.run_once()
    p2 = io.states[-1]['pick']
    assert supply_finds(io) == 1 and io.states[-1]['state'] == 'PICK_PLACE'
    assert p2['image_seq'] == 7 and p2['index'] == 4 and p2['reason'] != p1['reason']          # 같은 블록의 다음 잡기


def test_정지는_순위를_유지하고_정리_확인_취소면_빼고_IDLE_까지_안_싣는다():
    m, io = make()
    io.find_script = [(True, '', scene(7))]
    start(m)
    drive(m, 'PICK_PLACE')
    m.on_safety(dict(SAFE_STOP))
    m.run_once()
    assert m.state == 'STOPPED' and io.states[-1]['pick']['image_seq'] == 7       # 정지만으로는 조립이 안 끝났다
    n = len(io.states)
    assert m.command('cancel') == (True, '')                                      # [초기화] — 이번 조립을 버린다
    m.on_safety(dict(SAFE_OK))
    drive(m, 'IDLE')
    assert len(io.states) > n and all('pick' not in s for s in io.states[n:])


def test_ERROR_에서_취소하면_IDLE_알림에서_뺀다():
    m, io = make()
    io.find_script = [(True, '', scene(7))]
    start(m)
    drive(m, 'PICK_PLACE')
    m._to_error('시험: 사람 확인')
    assert io.states[-1]['state'] == 'ERROR' and io.states[-1]['pick']['image_seq'] == 7
    assert m.command('cancel') == (True, '')
    assert io.states[-1]['state'] == 'IDLE' and 'pick' not in io.states[-1]
