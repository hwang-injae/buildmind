"""BlockFinder 시험 (pytest, ROS · 로봇 없음) — W086 흩어진 블록 찾기 ①, SDD §6.11 ②.

① 합성: 공급 관측 자세(observe_supply)의 실측 카메라(내부 파라미터 · T_base2cam = W114-v2 s06_001 자세 json 값)로
   블록 상자를 광선 추적해 깊이 영상 + 블록 번호 영상을 만든다. 정답 마스크(블록 번호 영상)를 넣어 뒷단(blocks_from_masks)만 보고,
   눕힘 1개 · 2개는 예비 마스크(masks_from_edges, cv2 필요)로도 본다.
② 실측(있을 때만): ~/Downloads/W114-v2(또는 환경 변수 D2_W114_DIR)의 흩뿌린 장면 몇 장에 예비 마스크 → 뒷단을 돌려 결과를 찍는다(pytest -s).
   D2_FIND_PREVIEW_DIR 를 주면 윤곽 · 번호 그림을 그 폴더에 저장한다(저장소 안에 두지 말 것). 정답 라벨이 로컬에 없어 판정은 하지 않는다.
CI 에는 cv2 가 없다 — cv2 가 필요한 시험은 importorskip 으로 건너뛴다(뒷단 시험은 numpy 만).
"""
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.block_finder import (LABEL_SCALE, STATE_BGR, TARGET_BGR, BlockFinder, _in_area, block_label, tilt_deg, blocks_from_masks,
                                   design_top_corners, draw_found, draw_workbench, finder_cfg, masks_from_edges, part_short_name,
                                   pick_redraw_key, placed_note, wrap_deg)

ROBOT_YAML = Path(__file__).resolve().parents[2] / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml'
CFG = finder_cfg(yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8')))
SIZE = CFG['size_m']
TABLE = CFG['table_z_m']
# 10/7 촬영(W114 · W114-v2)의 자세 json T_base2cam 은 그때 손목 보정값(작업대 −0.018 m 에 맞춰 고정)으로 계산돼 작업대가 약 −18 mm 로 나온다.
# 10/11 E-85(#123)로 table_z_m 이 0.000 이 되어도 그 자료는 그 기준으로 돌린다 — 아니면 작업대를 못 찾아(±6 mm 밖) s04_002 블록 8개 → 1개.
OLD_CAPTURE_TABLE_Z_M = -0.018
CFG_OLD = {**CFG, 'table_z_m': OLD_CAPTURE_TABLE_Z_M}
# W114-v2 s06_001_a_pose.json (observe_supply 자세, 10/7) — 카메라는 작업대 위 약 496 mm, 화면 긴 쪽 = base x
INTR = (603.2089233398438, 602.9675903320312, 319.44287109375, 246.93531799316406)
T_B2C = np.array([[0.999833, 0.018095, -0.002353, 0.389811],
                  [0.018148, -0.99953, 0.024701, 0.231348],
                  [-0.001905, -0.02474, -0.999692, 0.477916],
                  [0.0, 0.0, 0.0, 1.0]])
H, W = 480, 640
POS_TOL_MM, YAW_TOL_DEG = 3.0, 3.0
TILT_KEYS = {'center_top_z_m', 'normal'}   # tilted 블록에만 더하는 칸(10/13)
TILT_TOP_TOL_MM = 3.0     # 기울어진 블록 윗면 가장 높은 점: 평면의 98 % 높이 · 가장자리 EDGE_PX 깎기만큼 정답(모서리)보다 조금 낮다


# ---------------- 합성 장면 ----------------
def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# 블록 축(열 0 = LENGTH, 1 = WIDTH, 2 = THICKNESS) → base. up 축이 z 로 가게 둔다. 윗면 긴 변이 base x 를 향하게(yaw 0) 맞춘다.
R_UP = {
    'THICKNESS': np.eye(3),
    'WIDTH': np.array([[1.0, 0, 0], [0, 0, -1], [0, 1, 0]]),     # 폭 축이 위 · 길이 축이 x
    'LENGTH': np.array([[0, 1.0, 0], [0, 0, 1], [1, 0, 0]]),      # 길이 축이 위 · 폭 축(윗면 25 mm 변)이 x
}


def block(x_mm, y_mm, yaw_deg, up='THICKNESS', base_mm=0.0):
    """작업면(또는 base_mm 높이 받침) 위에 바로 놓인 블록 상자: (중심 m, 회전 3x3, 반치수 m)."""
    R = rz(math.radians(yaw_deg)) @ R_UP[up]
    hz = SIZE[['LENGTH', 'WIDTH', 'THICKNESS'].index(up)] / 2
    return (np.array([x_mm / 1000, y_mm / 1000, TABLE + base_mm / 1000 + hz]), R, np.array(SIZE) / 2)


def render(boxes):
    """상자들을 실측 카메라로 광선 추적 → (깊이 m (H, W), 블록 번호 (H, W) — 작업면 −1)."""
    v, u = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(u - INTR[2]) / INTR[0], (v - INTR[3]) / INTR[1], np.ones((H, W))], axis=-1)
    R, o = T_B2C[:3, :3], T_B2C[:3, 3]
    d = d_cam @ R.T                                   # 광선 방향(base). 카메라 z 성분이 1 이라 t = 카메라 깊이
    t_best = (TABLE - o[2]) / d[..., 2]
    idx = np.full((H, W), -1)
    for i, (c, Rb, h) in enumerate(boxes):
        o_l = Rb.T @ (o - c)
        d_l = d @ Rb
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (-h - o_l) / d_l, (h - o_l) / d_l
            tmin = np.max(np.minimum(t1, t2), axis=-1)
            tmax = np.min(np.maximum(t1, t2), axis=-1)
            hit = (tmax >= tmin) & (tmin > 0) & (tmin < t_best)
        t_best[hit] = tmin[hit]
        idx[hit] = i
    return t_best, idx


def color_of(idx):
    """합성 컬러: 작업면 밝은 회색, 블록은 나무색(블록마다 밝기 조금 다르게)."""
    img = np.full((H, W, 3), 225, np.uint8)
    for i in range(idx.max() + 1):
        img[idx == i] = (120 + 8 * i, 170 + 5 * i, 205)
    return img


def largest_piece(m):
    """마스크의 가장 큰 4-연결 조각(가려져 두 조각이 된 아래 블록 — E-66 라벨 규칙: 큰 조각 하나만)."""
    lab = np.zeros(m.shape, int)
    n = 0
    for v0, u0 in zip(*np.nonzero(m)):
        if lab[v0, u0]:
            continue
        n += 1
        stack = [(v0, u0)]
        lab[v0, u0] = n
        while stack:
            v, u = stack.pop()
            for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = v + dv, u + du
                if 0 <= a < H and 0 <= b < W and m[a, b] and not lab[a, b]:
                    lab[a, b] = n
                    stack.append((a, b))
    sizes = [(lab == k).sum() for k in range(1, n + 1)]
    return lab == (int(np.argmax(sizes)) + 1)


def run_truth(boxes, masks=None):
    """정답 마스크(블록 번호 영상)로 뒷단만 돌린다. 출력: (블록 목록, info, 마스크 번호 → 블록 dict)."""
    depth, idx = render(boxes)
    if masks is None:
        masks = [idx == i for i in range(len(boxes))]
    info = {}
    out = blocks_from_masks(masks, depth, INTR, T_B2C, CFG, info=info)
    return out, info, dict(zip(info['mask_index'], out))


def check_pose(b, x_mm, y_mm, yaw_deg, up):
    """자리 ±3 mm · yaw ±3°(180° 대칭) · 자세."""
    assert b['up'] == up, b
    assert abs(b['x_m'] * 1000 - x_mm) <= POS_TOL_MM, b
    assert abs(b['y_m'] * 1000 - y_mm) <= POS_TOL_MM, b
    assert abs(wrap_deg(b['yaw_deg'] - yaw_deg)) <= YAW_TOL_DEG, b
    assert -90.0 <= b['yaw_deg'] < 90.0


def check_schema(blocks):
    """IRD find_blocks 칸 그대로 · JSON 직렬화 가능(NaN 없음) · 윗면 높이 순 · 틈 키 = 수평 두 축.
    tilted 블록에만 center_top_z_m · normal(단위 길이, 위쪽) 두 칸이 더 있다 — 기울지 않은 블록은 정확히 원래 칸."""
    json.dumps(blocks, allow_nan=False)
    want = {'x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'clear', 'gap_mm', 'overlap', 'tilted', 'confidence'}
    for b in blocks:
        assert set(b) == (want | TILT_KEYS if b['tilted'] else want)
        if b['tilted']:
            n = b['normal']
            assert len(n) == 3 and abs(math.hypot(*n) - 1.0) < 1e-3 and n[2] > 0
        axes = {'THICKNESS': {'LENGTH', 'WIDTH'}, 'WIDTH': {'LENGTH', 'THICKNESS'}, 'LENGTH': {'WIDTH', 'THICKNESS'}}[b['up']]
        assert set(b['gap_mm']) == axes and set(b['clear']) == axes
        assert b['overlap'] in ('none', 'top', 'under') and isinstance(b['tilted'], bool)
        assert 0.0 <= b['confidence'] <= 1.0
    tops = [b['top_z_m'] for b in blocks]
    assert tops == sorted(tops, reverse=True)


# ---------------- 합성 — 정답 마스크로 뒷단 ----------------
# 10/11 박진용 교시 테이프 안쪽 직사각형(robot.yaml supply_area_m) — 손 계산 점이 yaml 값이 바뀌어도 이 숫자와 맞게 여기 적는다
TAPE = ((0.1737, 0.5923), (0.1221, 0.3123))


def test_area_rect_hand_points():
    """흩뿌릴 영역 = supply_area_m 직사각형(x 173.7 ~ 592.3 · y 122.1 ~ 312.3 mm). 네 변 0.5 mm 안 · 밖, 가운데, 깊이 없는 점.
    옛 고정 네모(x 183 ~ 586 · y 142 ~ 337)와 다른 띠도 본다 — 로봇 쪽 y 130 은 이제 안, y 330 은 이제 밖."""
    pts = {(174.2, 215.0): True, (173.2, 215.0): False,       # 왼 변
           (591.8, 215.0): True, (592.8, 215.0): False,       # 오른 변
           (383.0, 122.6): True, (383.0, 121.6): False,       # 아래(로봇 쪽) 변
           (383.0, 311.8): True, (383.0, 312.8): False,       # 위 변
           (383.0, 215.0): True,                              # 가운데
           (380.0, 130.0): True, (380.0, 330.0): False,       # 옛 고정 네모와 다른 띠
           (float('nan'), 215.0): False, (383.0, float('nan')): False}   # 깊이 없는 픽셀은 밖
    P = np.array([[[x / 1000, y / 1000, 0.0] for (x, y) in pts]])
    got = _in_area(P, {'area_m': TAPE})[0]
    assert list(got) == list(pts.values()), dict(zip(pts, got))


def test_finder_cfg_reads_supply_area():
    """finder_cfg 는 robot.yaml supply_area_m {x_m, y_m} 를 그대로 쓴다 — 키가 없으면 KeyError, 형식이 틀리면 ValueError(노드 시작 때 드러남)."""
    robot = yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8'))
    area = robot['supply_area_m']
    assert CFG['area_m'] == (tuple(map(float, area['x_m'])), tuple(map(float, area['y_m'])))
    with pytest.raises(KeyError):
        finder_cfg({k: v for k, v in robot.items() if k != 'supply_area_m'})
    for bad in ([[0.17, 0.12], [0.59, 0.11], [0.59, 0.31], [0.17, 0.31]],     # 옛 꼭짓점 목록 꼴(#126 첫 커밋)
                {'x_m': [0.17, 0.59]},                                       # y_m 없음
                {'x_m': [0.17, 0.59, 0.6], 'y_m': [0.12, 0.31]},             # 세 수
                {'x_m': [0.59, 0.17], 'y_m': [0.12, 0.31]},                  # 최소 > 최대
                {'x_m': [0.17, 0.59], 'y_m': ['a', 0.31]}):                  # 수가 아님
        with pytest.raises(ValueError):
            finder_cfg({**robot, 'supply_area_m': bad})


def test_flat_single_yaw12():
    out, _, _ = run_truth([block(420, 210, 12)])
    check_schema(out)
    b, = out
    check_pose(b, 420, 210, 12, 'THICKNESS')
    assert abs(b['top_z_m'] - (TABLE + SIZE[2])) < 0.002
    assert b['clear'] == {'LENGTH': True, 'WIDTH': True} and b['overlap'] == 'none' and not b['tilted']


def test_edge_single():
    out, _, _ = run_truth([block(300, 260, -30, 'WIDTH')])
    check_schema(out)
    b, = out
    check_pose(b, 300, 260, -30, 'WIDTH')
    assert abs(b['top_z_m'] - (TABLE + SIZE[1])) < 0.002
    assert all(b['clear'].values()) and b['overlap'] == 'none' and not b['tilted']


def test_stand_single_yaw_by_width_edge():
    """세움 — yaw 는 윗면 25 mm 변(WIDTH) 기준(10/8 E-62). 카메라 가운데에서 약 100 mm 떨어져 옆면이 보여도 윗면만 쓴다."""
    out, _, _ = run_truth([block(480, 280, 45, 'LENGTH')])
    check_schema(out)
    b, = out
    check_pose(b, 480, 280, 45, 'LENGTH')
    assert abs(b['top_z_m'] - (TABLE + SIZE[0])) < 0.002
    assert all(b['clear'].values()) and b['overlap'] == 'none' and not b['tilted']


def _pair_8mm():
    """눕힘 두 개를 긴 변끼리 8 mm 틈으로 나란히(yaw 12°)."""
    yaw = math.radians(12)
    off = SIZE[1] * 1000 + 8.0                       # 중심 사이 = 폭 + 8 mm
    vx, vy = -math.sin(yaw) * off, math.cos(yaw) * off
    return [block(390, 200, 12), block(390 + vx, 200 + vy, 12)], (390 + vx, 200 + vy)


def test_two_flat_8mm_gap():
    boxes, (x2, y2) = _pair_8mm()
    out, _, by = run_truth(boxes)
    check_schema(out)
    check_pose(by[0], 390, 200, 12, 'THICKNESS')
    check_pose(by[1], x2, y2, 12, 'THICKNESS')
    for b in out:
        assert abs(b['gap_mm']['WIDTH'] - 8.0) <= 2.0, b      # 2 mm 격자 — ±1.4 mm + 여유
        assert b['clear'] == {'LENGTH': True, 'WIDTH': False}
        assert b['overlap'] == 'none' and not b['tilted']


def test_tilted_on_other_block():
    """B 가 한쪽 끝은 작업면, 다른 쪽은 A 의 끝 모서리에 걸쳐 기울어짐(E-43) → B tilted · top, A under(걸친 블록도 빼야 함)."""
    L, _, T = (s * 1000 for s in SIZE)
    a_x, a_y = 390.0, 230.0
    a_left = a_x - L / 2                             # A 의 왼쪽 끝 (x mm)
    x_low = a_left - 52.0                            # B 아래 모서리가 작업면에 닿는 곳
    th = math.atan2(T, a_left - x_low)               # B 밑면이 A 왼쪽 위 모서리에 닿는 기울기
    # B 중심 = 밑면 가운데 + 두께/2 × 윗면 쪽 법선(x-z 평면, 길이 축이 +x 로 올라감)
    bx = x_low + (L / 2) * math.cos(th) - (T / 2) * math.sin(th)
    bz = (L / 2) * math.sin(th) + (T / 2) * math.cos(th)
    b_y = a_y + 5.0
    B = (np.array([bx / 1000, b_y / 1000, TABLE + bz / 1000]), ry(-th), np.array(SIZE) / 2)
    out, info, by = run_truth([block(a_x, a_y, 0), B])
    check_schema(out)
    b = by[1]
    assert b['tilted'] and b['up'] == 'THICKNESS' and b['overlap'] == 'top', (b, info['detail'])
    top_cx = bx - (T / 2) * math.sin(th)              # 윗면 가운데의 x (기울어진 만큼 안쪽)
    assert abs(b['x_m'] * 1000 - top_cx) <= POS_TOL_MM and abs(b['y_m'] * 1000 - b_y) <= POS_TOL_MM
    assert abs(wrap_deg(b['yaw_deg'])) <= YAW_TOL_DEG
    # tilted 의 top_z_m = 윗면 가장 높은 점(E-68 장애물 상자, 10/11) — 윗면 가운데보다 (L/2)·sin θ(≈ 10 mm) 높다
    top_c = bz + (T / 2) * math.cos(th)              # 윗면 가운데 높이(작업면 위 mm)
    top_hi = top_c + (L / 2) * math.sin(th)          # 윗면 높은 끝(A 위로 올라간 쪽)
    got = (b['top_z_m'] - TABLE) * 1000
    assert abs(got - top_hi) <= TILT_TOP_TOL_MM and got > top_c + 5.0, (got, top_c, top_hi)
    # center_top_z_m = 윗면 가운데 평면 높이 · normal = 윗면 법선 — B 는 길이 축이 +x 로 θ 올라가 법선이 (−sin θ, 0, cos θ)
    assert abs((b['center_top_z_m'] - TABLE) * 1000 - top_c) <= TILT_TOP_TOL_MM, (b['center_top_z_m'], top_c)
    assert np.allclose(b['normal'], [-math.sin(th), 0.0, math.cos(th)], atol=0.03), (b['normal'], math.degrees(th))
    a = by[0]
    assert a['overlap'] == 'under' and not a['tilted'] and a['up'] == 'THICKNESS', (a, info['detail'])


def _sideways(x_mm, y_mm, deg):
    """긴 축(x) 둘레로 deg 만큼 옆으로 기운 눕힌 블록 — 아래쪽 긴 모서리가 작업면에 닿음(폭 축이 +y 로 올라감)."""
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    rx = np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])
    W, T = SIZE[1], SIZE[2]
    return (np.array([x_mm / 1000, y_mm / 1000, TABLE + (W / 2) * s + (T / 2) * c]), rx, np.array(SIZE) / 2)


def test_sideways_tilt_caught_by_angle():
    """옆으로 8° 기운 눕힌 블록 — 25 mm 폭에서 높이 차는 약 3.5 mm(5 mm 미만)라 예전엔 반듯하다고 봤다. 윗면 각도 ≥ 5° 로 tilted(10/14 PL B1).
    3° 는 반듯(보통 집기 한계 5° 안). 각도 기준은 눕힌 넓은 윗면에서만 쓴다."""
    out, info, by = run_truth([_sideways(420, 210, 8.0)])
    check_schema(out)
    b = by[0]
    assert b['tilted'] and b['up'] == 'THICKNESS', (b, info['detail'])
    nz = b['normal'][2]
    assert abs(math.degrees(math.acos(nz)) - 8.0) <= 1.5, b['normal']
    out, info, by = run_truth([_sideways(420, 210, 3.0)])
    assert not by[0]['tilted'], (by[0], info['detail'])


def test_stacked_cross():
    """포개짐: A 위에 B 가 90° 엇갈려 얹힘 → B up THICKNESS · top · 윗면 = 두께 2개, A under(보이는 큰 조각만 마스크 — E-66)."""
    T = SIZE[2] * 1000
    # y 250: 세로(90°) B 가 y 212.5 ~ 287.5 로 테이프(supply_area_m, 위 변 y 312.3) 안 — 10/11 전 y 290 은 327.5 까지 뻗어 테이프를 넘었다
    boxes = [block(330, 250, 0), block(345, 250, 90, base_mm=T)]
    _, idx = render(boxes)
    masks = [largest_piece(idx == 0), idx == 1]
    out, info, by = run_truth(boxes, masks)
    check_schema(out)
    b = by[1]
    check_pose(b, 345, 250, 90, 'THICKNESS')
    assert b['overlap'] == 'top' and not b['tilted']
    assert abs(b['top_z_m'] - (TABLE + 2 * SIZE[2])) < 0.002
    assert out[0] is b                                # 윗면 높이 순 — 위 블록이 먼저
    assert by[0]['overlap'] == 'under', (by[0], info['detail'])


def test_clipped_at_tape_edge():
    """테이프(supply_area_m) 위 변(y 312.3 mm)을 걸친 눕힌 블록(y 297.5 ~ 322.5): 잘림 판정 영역은 손가락 여유(clip_margin_m)만큼 넓어
    none(10/13 004 실기 — 윤곽이 다 보이는데 귀퉁이 3 mm 가 테이프 밖이라 빠져 공급 대기). 여유를 0 으로 두면 예전처럼(10/11) 영역 밖으로
    이어지니 under(집지 않음). 테이프 안(y 280)은 none, 영역을 끄면(area_m None) 걸친 블록도 none — 영상 테두리 탓이 아님을 본다."""
    depth, idx = render([block(383, 310, 0)])
    assert CFG['clip_margin_m'] > 0.0125
    on_edge = blocks_from_masks([idx == 0], depth, INTR, T_B2C, CFG)
    no_margin = blocks_from_masks([idx == 0], depth, INTR, T_B2C, {**CFG, 'clip_margin_m': 0.0})
    no_area = blocks_from_masks([idx == 0], depth, INTR, T_B2C, {**CFG, 'area_m': None})
    depth2, idx2 = render([block(383, 280, 0)])
    inside = blocks_from_masks([idx2 == 0], depth2, INTR, T_B2C, CFG)
    assert on_edge[0]['overlap'] == 'none', on_edge
    assert no_margin[0]['overlap'] == 'under' and no_margin[0]['confidence'] <= 0.2, no_margin
    assert no_area[0]['overlap'] == 'none', no_area
    assert inside[0]['overlap'] == 'none', inside


def test_clipped_at_image_border():
    """영상 왼쪽 테두리에 잘린 블록(영역 밖이기도 함) → 자세 · yaw 를 못 믿으니 under + confidence ≤ 0.2 (10/8 PL)."""
    out, info, _ = run_truth([block(140, 230, 0)])
    b, = out
    assert b['overlap'] == 'under' and b['confidence'] <= 0.2, (b, info['detail'])


def test_empty_and_bad_masks():
    """빈 마스크 · 크기 다른 마스크 · 깊이 없는 영상은 빼고 빈 목록(예외 없음)."""
    depth, _ = render([block(420, 210, 0)])
    info = {}
    out = blocks_from_masks([np.zeros((H, W), bool), np.ones((10, 10), bool)], depth, INTR, T_B2C, CFG, info=info)
    assert out == [] and len(info['skipped']) == 2
    assert blocks_from_masks([np.ones((H, W), bool)], np.zeros((H, W)), INTR, T_B2C, CFG) == []


# ---------------- 합성 — 예비 마스크(엣지 + 깊이) ----------------
def test_edges_single_flat():
    pytest.importorskip('cv2')
    depth, idx = render([block(420, 210, 12)])
    out = BlockFinder(CFG).find(color_of(idx), depth, INTR, T_B2C)
    check_schema(out)
    b, = out
    check_pose(b, 420, 210, 12, 'THICKNESS')
    assert all(b['clear'].values())


def test_edges_two_flat_8mm():
    pytest.importorskip('cv2')
    boxes, (x2, y2) = _pair_8mm()
    depth, idx = render(boxes)
    masks = masks_from_edges(color_of(idx), depth, INTR, T_B2C, CFG)
    assert len(masks) == 2
    out = blocks_from_masks(masks, depth, INTR, T_B2C, CFG)
    check_schema(out)
    got = sorted(out, key=lambda b: b['y_m'])
    check_pose(got[0], 390, 200, 12, 'THICKNESS')
    check_pose(got[1], x2, y2, 12, 'THICKNESS')
    for b in got:
        assert b['clear'] == {'LENGTH': True, 'WIDTH': False} and abs(b['gap_mm']['WIDTH'] - 8.0) <= 3.0


# ---------------- 실측 장면 (있을 때만, 판정 없이 출력) ----------------
REAL_DIR = Path(os.environ.get('D2_W114_DIR', Path.home() / 'Downloads' / 'W114-v2'))
REAL_SCENES = ['s07_040_a', 's06_010_a', 's08_040_a', 's08_030_a']   # 드문 · 보통 · 세움 많음 · 빈 장면


def _draw(color, masks, info, blocks, path, cv2):
    """윤곽(판정 색) + 번호 + 자세 · yaw · 틈 글자. 초록 = 집을 수 있음(none · 기울지 않음 · 틈 하나 이상), 주황 = top, 빨강 = under · tilted."""
    img = color.copy()
    for n, (mi, b) in enumerate(zip(info['mask_index'], blocks)):
        m = masks[mi].astype(np.uint8)
        if b['tilted'] or b['overlap'] == 'under':
            col = (0, 0, 255)
        elif b['overlap'] == 'top':
            col = (0, 140, 255)
        else:
            col = (0, 200, 0) if any(b['clear'].values()) else (200, 200, 0)
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cs, -1, col, 2)
        vs, us = np.nonzero(m)
        cv2.putText(img, str(n), (int(us.mean()) - 6, int(vs.mean()) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    for i in info.get('skipped', []):
        m = masks[i[0]].astype(np.uint8) if i[0] < len(masks) else None
        if m is not None:
            cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cs, -1, (128, 128, 128), 1)
    legend = np.full((24 + 18 * len(blocks), img.shape[1], 3), 255, np.uint8)
    cv2.putText(legend, f'table_z {info["table_z_m"] * 1000:.1f} mm  blocks {len(blocks)}', (4, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    for n, b in enumerate(blocks):
        g = ' '.join(f'{k[0]}{v:.0f}' for k, v in b['gap_mm'].items())
        txt = (f'{n}: {b["up"][:5]} yaw {b["yaw_deg"]:+.0f} top {b["top_z_m"] * 1000:+.0f}mm gap {g} '
               f'{b["overlap"]}{" TILT" if b["tilted"] else ""} c{b["confidence"]:.2f}')
        cv2.putText(legend, txt, (4, 34 + 18 * n), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    cv2.imwrite(str(path), np.vstack([img, legend]))


@pytest.mark.parametrize('name', REAL_SCENES)
def test_real_scene_preview(name):
    """실측 흩뿌린 장면: 예비 마스크 → 뒷단. 결과를 찍고(pytest -s) D2_FIND_PREVIEW_DIR 가 있으면 그림 저장. 자세 json 의 T_base2cam 을 쓴다."""
    cv2 = pytest.importorskip('cv2')
    stem = REAL_DIR / name
    if not Path(f'{stem}_color.png').is_file():
        pytest.skip(f'실측 데이터 없음: {stem}_color.png')
    pose = json.loads(Path(f'{stem}_pose.json').read_text())
    ii = pose['intrinsics_px']
    intr = (ii['fx'], ii['fy'], ii['ppx'], ii['ppy'])
    T = np.array(pose['T_base2cam_m'])
    color = cv2.imread(f'{stem}_color.png')
    depth = cv2.imread(f'{stem}_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    finder = BlockFinder(CFG_OLD)
    blocks = finder.find(color, depth, intr, T)
    check_schema(blocks)
    info = finder.last_info
    print(f'\n[{name}] 작업면 {info["table_z_m"] * 1000:.1f} mm · 마스크 {len(finder.last_masks)} · 블록 {len(blocks)}'
          f' · 빠짐 {len(info["skipped"])}')
    for n, (mi, b) in enumerate(zip(info['mask_index'], blocks)):
        d = info['detail'][mi]
        print(f'  {n:2d} {b["up"]:9s} yaw {b["yaw_deg"]:+6.1f} ({b["x_m"] * 1000:5.0f},{b["y_m"] * 1000:5.0f}) '
              f'top {b["top_z_m"] * 1000:+5.1f} gap {b["gap_mm"]} {b["overlap"]:5s} tilt {b["tilted"]!s:5s} '
              f'c {b["confidence"]:.2f} | 윗면 {d["long_mm"]:.0f}x{d["short_mm"]:.0f} h {d["h_c"]:.1f} 기울 {d["tilt_mm"]:.1f}'
              f' 면적비 {d["area_ratio"]:.2f}')
    out_dir = os.environ.get('D2_FIND_PREVIEW_DIR')
    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        _draw(color, finder.last_masks, info, blocks, Path(out_dir) / f'{name}_find.png', cv2)


REAL1_DIR = Path(os.environ.get('D2_W114_V1_DIR', Path.home() / 'Downloads' / '데이터셋' / 'W114'))   # 첫 촬영(s01~s05)


def test_real_separated_blocks_have_clear_axis():
    """실측 회귀(있을 때만): s04_002 는 블록 7개가 흩어져 있다(서로 붙은 건 옆으로 8~10 mm 떨어진 한 쌍뿐).
    예비 마스크 → 뒷단에서 덮이지 않고 기울지 않은 블록 7개 중 6개 이상이 한 축 이상 clear 여야 한다.
    (10/8 고치기 전: 내 블록 둘레 깊이 번짐 · 마스크보다 긴 윗면 끝이 장애물로 세져 2개뿐 — 틈 4~7 mm)"""
    cv2 = pytest.importorskip('cv2')
    stem = REAL1_DIR / 's04_002_a'
    if not Path(f'{stem}_color.png').is_file():
        pytest.skip(f'실측 데이터 없음: {stem}_color.png')
    pose = json.loads(Path(f'{stem}_pose.json').read_text())
    ii = pose['intrinsics_px']
    color = cv2.imread(f'{stem}_color.png')
    depth = cv2.imread(f'{stem}_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    blocks = BlockFinder(CFG_OLD).find(color, depth, (ii['fx'], ii['fy'], ii['ppx'], ii['ppy']), np.array(pose['T_base2cam_m']))
    free = [b for b in blocks if b['overlap'] == 'none' and not b['tilted']]
    assert len(free) == 7, blocks
    assert sum(any(b['clear'].values()) for b in free) >= 6, [b['gap_mm'] for b in free]


def test_기울어진_블록_평면은_사각형_안쪽_점으로_다시_맞추고_점이_모자라면_칸이_없다():
    """_tilt_keys = one_pick tilt_pose 방법(블록 사각형 안쪽 · 작업면 + 4 mm ~ top + 5 mm 점, 3 mm 잔차 두 번, 50점 미만이면 칸 없음 — 10/13 b1 대조 #5)."""
    from d2_vision.block_finder import _tilt_keys
    th = math.radians(10.0)
    xs, ys = np.meshgrid(np.linspace(-0.03, 0.03, 30), np.linspace(-0.008, 0.008, 9))
    z = 0.02 + np.tan(th) * xs                                  # 길이 축(+x)으로 10° 올라간 윗면, 가운데 20 mm
    P = np.stack([0.4 + xs, 0.2 + ys, z], axis=-1)
    cfg = {'size_m': (0.07445, 0.0248, 0.0148)}
    b = {'x_m': 0.4, 'y_m': 0.2, 'yaw_deg': 0.0, 'top_z_m': float(z.max())}
    k = _tilt_keys(b, P, 0.0, cfg)
    assert k['center_top_z_m'] == pytest.approx(0.02, abs=1e-4)
    assert np.allclose(k['normal'], [-math.sin(th), 0.0, math.cos(th)], atol=1e-3)
    assert _tilt_keys(b, P[:2, :5], 0.0, cfg) == {}              # 10점 — 50점 미만


def test_block_label_names_pose_and_tilt_angle():
    """그림 글자: 자세 이름 FLAT · EDGE · STAND(10/14 20시 PL) · 번호 · yaw 없음 · 기울었을 때만 ' TILT n'(n = 윗면 법선과 수직 사이 각 °,
    10/14 22시 PL — yaw 가 아니라 기울어진 각도) · normal 이 없으면 ' TILT' 만 · ASCII."""
    blk = {'up': 'THICKNESS', 'yaw_deg': 12.4, 'tilted': False}
    assert block_label(blk) == 'FLAT'
    assert block_label({**blk, 'up': 'WIDTH'}) == 'EDGE'
    th = math.radians(8.2)
    assert block_label({**blk, 'up': 'LENGTH', 'tilted': True, 'normal': [math.sin(th), 0.0, math.cos(th)]}) == 'STAND TILT 8'
    assert block_label({**blk, 'tilted': True}) == 'FLAT TILT'
    assert tilt_deg({'normal': [0.0, 0.0, 2.0]}) == 0 and tilt_deg({}) is None
    assert block_label(blk).isascii()


def test_draw_found_rank_badges_and_pick_fill():
    """순위(ranks — 블록 번호 1부터, 우선순위 차례)가 있으면 블록 가운데에 번호 동그라미(흰 바탕), 고른 블록(pick)은 윤곽 색으로 반투명 채우기 ·
    굵은 테두리. 순위가 없으면 동그라미 · 채우기 없음(find 직후 그림)."""
    pytest.importorskip('cv2')
    color = np.full((H, W, 3), 60, np.uint8)
    m1, m2 = np.zeros((H, W), bool), np.zeros((H, W), bool)
    m1[100:140, 100:220] = True
    m2[300:340, 350:470] = True
    blks = [{'up': 'THICKNESS', 'overlap': 'none', 'tilted': False}] * 2
    plain = draw_found(color, blks, [m1, m2], [0, 1])
    ranked = draw_found(color, blks, [m1, m2], [0, 1], ranks=[2, 1], pick=2)
    inside = lambda img, m: img[m].reshape(-1, 3)
    assert not (inside(plain, m1) == 255).all(axis=1).any()                  # 순위 전 — 동그라미 없음
    assert (inside(ranked, m1) == 255).all(axis=1).any() and (inside(ranked, m2) == 255).all(axis=1).any()
    core1, core2 = np.zeros((H, W), bool), np.zeros((H, W), bool)
    core1[104:110, 104:130] = True                                           # 동그라미 · 테두리 밖 안쪽 자리
    core2[304:310, 354:380] = True
    assert (ranked[core1] == 60).all() and (ranked[core2] != 60).all()       # 2번(고른 블록)만 채움
    assert ranked[core2][0][1] > 60                                          # 초록(none) 쪽으로 물듦


def test_draw_found_badge_moves_off_label():
    """10/15 PL '가'(박진용 실기): 얇은 기울어진 블록은 가운데 동그라미가 윤곽 바로 위 'FLAT TILT n' 글자를 덮었다 — 동그라미는
    글자와 안 겹치는 블록 안 자리로 비킨다(글자 자리 · 순서는 그대로)."""
    pytest.importorskip('cv2')
    color = np.full((H, W, 3), 60, np.uint8)
    m = np.zeros((H, W), bool)
    m[200:212, 260:380] = True                    # 높이 12 px — 가운데(206)에 두면 동그라미(반지름 ~10)가 윗변 위 글자까지 닿는다
    t = math.radians(12)
    blk = {'up': 'THICKNESS', 'overlap': 'none', 'tilted': True, 'normal': [0.0, math.sin(t), math.cos(t)]}
    plain = draw_found(color, [blk], [m], [0])
    ranked = draw_found(color, [blk], [m], [0], ranks=[1])
    lv, lu = np.nonzero((plain != 60).any(axis=2) & (np.arange(H)[:, None] < 198))   # 글자 픽셀(윗변 윤곽 테는 뺌)
    bv, bu = np.nonzero((ranked != plain).any(axis=2))                                # 동그라미 픽셀
    assert len(lv) and len(bv)
    assert not ((bu >= lu.min()) & (bu <= lu.max()) & (bv >= lv.min()) & (bv <= lv.max())).any()   # 글자 상자와 안 겹침
    assert m[int(round(bv.mean())), int(round(bu.mean()))]                             # 동그라미 가운데는 블록 안


@pytest.mark.parametrize('k', [0, 11])
def test_draw_found_label_inside_image_at_border(k):
    """오른쪽 아래 테두리에 닿은 블록도 긴 글자('STAND TILT')가 영상 밖으로 나가지 않는다(옛 고정 90 px 당김은 잘렸다)."""
    cv2 = pytest.importorskip('cv2')
    color = np.zeros((H, W, 3), np.uint8)
    m = np.zeros((H, W), bool)
    m[H - 20:, W - 30:] = True
    blk = {'up': 'LENGTH', 'yaw_deg': -45.0, 'overlap': 'none', 'tilted': True}
    masks = [m] * (k + 1)
    img = draw_found(color, [blk] * (k + 1), masks, list(range(k + 1)))
    txt = block_label(blk)
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 1)
    vs, us = np.nonzero((img == 255).all(axis=2))          # 흰 글자 픽셀(윤곽은 초록 · 글자 테두리는 검정)
    assert img.shape == color.shape and len(us)
    assert us.max() < W - 1 and vs.max() < H - 1 and us.min() > 0 and vs.min() > 0   # 테두리에 닿지 않음 = 잘리지 않음
    assert us.max() - us.min() + 1 >= 0.8 * tw              # 글자 폭이 거의 다 그려짐


def test_draw_found_label_above_outline():
    """글자는 블록 윤곽 위쪽에(블록을 덜 가리게, 10/14 20시 PL) — 가운데 블록이면 흰 글자 픽셀이 모두 마스크 윗변보다 위."""
    pytest.importorskip('cv2')
    color = np.zeros((H, W, 3), np.uint8)
    m = np.zeros((H, W), bool)
    m[200:240, 260:380] = True
    img = draw_found(color, [{'up': 'THICKNESS', 'yaw_deg': 3.0, 'overlap': 'none', 'tilted': False}], [m], [0])
    vs, us = np.nonzero((img == 255).all(axis=2))
    assert len(vs) and vs.max() < 200 and abs((us.min() + us.max()) / 2 - 319.5) <= 3     # 윗변 위 · 가로 가운데


# ---------------- 작업대 그림 · 고른 뒤 덧그림 (10/14 22시 PL — 박진용 안 ②, state pick) ----------------
# 작업대 바로 위 카메라(관측 자세처럼 카메라 z 가 아래로): 카메라 축 = base (x, −y, −z), (0.40, 0.0) 위 0.40 m
BENCH_INTR = (600.0, 600.0, 320.0, 240.0)
BENCH_T = np.array([[1.0, 0.0, 0.0, 0.40], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.40], [0.0, 0.0, 0.0, 1.0]])
FLAT = np.eye(3)                                  # 눕힘 — 길이 = base x, 두께 = 위


def _part(pid, x, y, state, rot=FLAT):
    """작업대 그림 부품 한 개(BlockChecker.parts 칸 + check_progress state) — 작업대 위 눕힌 블록 가운데."""
    return {'part_id': pid, 'center': np.array([x, y, SIZE[2] / 2]), 'rot': rot, 'state': state}


def test_design_top_corners_flat_and_standing():
    """설계 윗면 네 모서리: 눕힘 = 길이 × 폭, z = 두께 · 세움(길이 축이 위) = 두께 × 폭, z = 가운데 + 길이/2. 차례대로 이으면 사각형."""
    c = design_top_corners([0.4, 0.0, SIZE[2] / 2], FLAT, SIZE)
    assert np.allclose(c[:, 2], SIZE[2]) and np.allclose(np.ptp(c[:, 0]), SIZE[0]) and np.allclose(np.ptp(c[:, 1]), SIZE[1])
    stand = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])   # 열 = 블록 축(길이 → base +z)
    c = design_top_corners([0.4, 0.0, SIZE[0] / 2], stand, SIZE)
    assert np.allclose(c[:, 2], SIZE[0]) and np.allclose(np.ptp(c[:, 0]), SIZE[2]) and np.allclose(np.ptp(c[:, 1]), SIZE[1])
    assert np.allclose(c.mean(axis=0)[:2], [0.4, 0.0])


def test_part_short_name_and_placed_note():
    """짧은 이름 'LEG 1-3'(역할 + 그룹-부품, 앞 0 뺌 · 설계 이름 뺌) · 옵션 붙은 역할 · 꼴이 다르면 이름 그대로 · 한글은 '?' (cv2 는 한글 못 그림).
    다시 재기 글자: 잼 → base dx · dy mm, 못 잼 → NOT MEASURED + 거절 코드."""
    assert part_short_name('001_CHAIR_BENCH_V000_LEG_001_03') == 'LEG 1-3'
    assert part_short_name('LEG_001_03') == 'LEG 1-3'
    assert part_short_name('002_CHAIR_BACK_V012_BACK_REST_002_10') == 'BACK_REST 2-10'
    assert part_short_name('odd') == 'odd' and part_short_name('의자').isascii()
    c = (0.40, -0.05, 0.0075)
    assert placed_note('001_CHAIR_BENCH_V000_LEG_001_03', None, c, 'FEW_POINTS') == 'LEG 1-3 NOT MEASURED FEW_POINTS'
    assert placed_note('LEG_001_03', None, c) == 'LEG 1-3 NOT MEASURED'
    meas = {'x_m': 0.4021, 'y_m': -0.0508, 'top_z_m': 0.015, 'yaw_deg': 1.0, 'up': 'THICKNESS'}
    assert placed_note('LEG_001_03', meas, c) == 'LEG 1-3 base dx +2.1 dy -0.8 mm'


def test_draw_workbench_colours_by_state_and_copy():
    """check_progress 판정 색: present 초록 · occluded · unknown 주황 · absent 빨강(2 px) — 각 사각형 가운데가 손 계산 픽셀 근처 ·
    모르는 state 값도 주황 · 입력은 그대로 · 크기 그대로 · 카메라 뒤(카메라보다 높은) 부품은 건너뛴다(예외 없음)."""
    pytest.importorskip('cv2')
    color = np.full((H, W, 3), 90, np.uint8)
    parts = [_part('D_V000_LEG_001_01', 0.33, -0.06, 'present'), _part('D_V000_LEG_001_02', 0.47, -0.06, 'absent'),
             _part('D_V000_SEAT_001_01', 0.33, 0.06, 'occluded'), _part('D_V000_SEAT_001_02', 0.47, 0.06, 'weird'),
             {'part_id': 'D_V000_TOP_001_01', 'center': np.array([0.4, 0.0, 0.6]), 'rot': FLAT, 'state': 'present'}]
    img = draw_workbench(color, parts, BENCH_INTR, BENCH_T, SIZE)
    assert img.shape == color.shape and img.dtype == color.dtype and (color == 90).all()
    fx, fy, ppx, ppy = BENCH_INTR
    for p in parts[:4]:
        bgr = STATE_BGR.get(p['state'], STATE_BGR['unknown'])
        vs, us = np.nonzero((img == bgr).all(axis=2))
        Z = 0.40 - SIZE[2]
        hu, hv = fx * (p['center'][0] - 0.40) / Z + ppx, fy * (-p['center'][1]) / Z + ppy
        near = (np.abs(us - hu) < 80) & (np.abs(vs - hv) < 30)
        assert near.sum() > 100, p['state']                                  # 그 자리에 그 색 윤곽
    assert STATE_BGR['occluded'] == STATE_BGR['unknown'] == (0, 165, 255) and STATE_BGR['absent'] == (0, 0, 255)


def test_draw_workbench_labels_inside_image_and_skip_overlap():
    """부품 짧은 이름(흰 글자)은 영상 안에 — 영상 테두리 부품도 잘리지 않게 당긴다. 같은 자리 부품 둘은 글자를 겹쳐 쓰지 않는다
    (둘째는 사각형 아래로)."""
    cv2 = pytest.importorskip('cv2')
    color = np.zeros((H, W, 3), np.uint8)
    edge = _part('D_V000_LEG_001_01', 0.40 + 0.20, 0.0, 'present')       # 오른쪽 끝(u ≈ 632) — 사각형이 영상 밖으로 나감
    img = draw_workbench(color, [edge], BENCH_INTR, BENCH_T, SIZE)
    vs, us = np.nonzero((img == 255).all(axis=2))
    (tw, _), _ = cv2.getTextSize('LEG 1-1', cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 1)
    assert len(us) and us.max() < W - 1 and us.min() > 0 and vs.min() > 0 and vs.max() < H - 1
    assert us.max() - us.min() + 1 >= 0.8 * tw
    same = [_part('D_V000_LEG_001_01', 0.40, 0.0, 'present'), _part('D_V000_LEG_001_02', 0.40, 0.0, 'absent')]
    img = draw_workbench(color, same, BENCH_INTR, BENCH_T, SIZE)
    vs, _ = np.nonzero((img == 255).all(axis=2))
    assert vs.max() - vs.min() > 25                                          # 글자 두 줄이 위아래로 나뉨(같은 자리에 겹쳐 쓰지 않음)


def test_draw_workbench_assembly_target_measured_and_note():
    """A4 다시 재기 그림: 설계 자리(state None) = 빨간 굵은 목표 상자 · 잰 블록 = 초록 · 왼쪽 위 글자(note). 잰 블록이 없으면 빨간 상자만."""
    pytest.importorskip('cv2')
    color = np.full((H, W, 3), 90, np.uint8)
    target = _part('001_CHAIR_BENCH_V000_LEG_001_03', 0.40, 0.0, None)
    meas = {'x_m': 0.403, 'y_m': -0.002, 'top_z_m': SIZE[2], 'yaw_deg': 4.0, 'up': 'THICKNESS'}
    note = placed_note(target['part_id'], meas, target['center'])
    img = draw_workbench(color, [target], BENCH_INTR, BENCH_T, SIZE, measured=meas, note=note)
    assert (img == TARGET_BGR).all(axis=2).any() and (img == STATE_BGR['present']).all(axis=2).any()
    assert (img[:30, :200] == 255).all(axis=2).any()                         # 왼쪽 위 글자(흰)
    only = draw_workbench(color, [target], BENCH_INTR, BENCH_T, SIZE)
    assert (only == TARGET_BGR).all(axis=2).any() and not (only == STATE_BGR['present']).all(axis=2).any()
    assert (draw_workbench(color, [], BENCH_INTR, BENCH_T, SIZE) == color).all()


def test_pick_redraw_key_decides_once_per_pick():
    """state pick → 다시 그릴 열쇠: 기억한 그림 번호와 같고 지난번과 다를 때만. pick 없음 · null 번호 · 옛 번호 · 같은 pick · 기억 없음은 None.
    ranks 가 없으면 빈 목록. 꼴이 틀리면 ValueError(노드는 경고 한 번만)."""
    pick = {'image_seq': 3, 'index': 2, 'ranks': [2, 1, 3], 'reason': 'clear'}
    assert pick_redraw_key(3, None, pick) == (3, 2, (2, 1, 3))
    assert pick_redraw_key(3, (3, 2, (2, 1, 3)), pick) is None                 # 같은 pick 이 state 마다 다시 와도 한 번만
    assert pick_redraw_key(3, (3, 2, (2, 1, 3)), {**pick, 'index': 1}) == (3, 1, (2, 1, 3))
    assert pick_redraw_key(3, None, {**pick, 'image_seq': 2}) is None         # 그 사이 다시 찾음 — 옛 그림
    assert pick_redraw_key(None, None, pick) is None                          # 노드가 다시 켜져 그림 기억 없음
    assert pick_redraw_key(3, None, None) is None
    assert pick_redraw_key(3, None, {**pick, 'image_seq': None}) is None
    assert pick_redraw_key(3, None, {'image_seq': 3, 'index': 1}) == (3, 1, ())
    for bad in ('x', [], {**pick, 'image_seq': '3'}, {**pick, 'image_seq': True}, {**pick, 'index': None},
                {**pick, 'ranks': 'ab'}, {**pick, 'ranks': [1, 'a']}):
        with pytest.raises(ValueError):
            pick_redraw_key(3, None, bad)
