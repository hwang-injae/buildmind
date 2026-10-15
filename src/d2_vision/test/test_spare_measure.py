"""받침 위 블록 재기 시험 (pytest, ROS · 로봇 없음) — find_blocks {"area": "spare"} (spare_measure.py).

합성 점: 받침 윗면(spare_spots[0] z_m) 평면 + 블록 윗면 1 mm 격자(± 0.5 mm 잡음) + 옆면 몇 줄. 눕힘 · 옆세움 · 세움 · 빈 받침 · 점 모자람.
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.block_finder import (AXIS_GAP_PX, OFFSET_SCALE, TARGET_BGR, _axes_spot, base_axes_px, draw_spare, project_to_image, spare_label,
                                    spare_offset_big, spare_offset_text, top_face_corners)
from d2_vision.spare_measure import OPEN_GAP_MM, ideal_spare_block, measure_on_spare

ROBOT = yaml.safe_load((Path(__file__).resolve().parents[2] / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml').read_text(encoding='utf-8'))
SPOT = ROBOT['spare_spots'][0]
SIZE = ROBOT['block_actual_m']               # [길이, 폭, 두께] m
L, W, T = SIZE
SX, SY, SZ = SPOT['x_m'], SPOT['y_m'], SPOT['z_m']
KEYS = {'x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'clear', 'gap_mm', 'overlap', 'tilted', 'confidence', 'base_z_m'}
# up → (윗면 yaw 쪽 변, 다른 변, 높이) — 세움 yaw 는 폭(25 mm) 변 기준(block_finder 와 같음)
POSE = {'THICKNESS': (L, W, T), 'WIDTH': (L, T, W), 'LENGTH': (W, T, L)}


def scene(up, cx, cy, yaw_deg, step=0.001, seed=0, support=True):
    """받침 윗면 점 + 블록(자세 up, 가운데 cx · cy, yaw) 윗면 · 옆면 점 (N, 3) m."""
    rng = np.random.default_rng(seed)
    pts = []
    if support:                                  # 받침 윗면 80 × 80 mm(블록 밑까지 — 위에서는 가려지지만 잡음 시험으로 둔다)
        g = np.arange(-0.04, 0.04, 0.002)
        X, Y = np.meshgrid(g, g)
        pts.append(np.c_[SX + X.ravel(), SY + Y.ravel(), np.full(X.size, SZ)])
    a, b, h = POSE[up]
    s, t = np.meshgrid(np.arange(-a / 2, a / 2, step), np.arange(-b / 2, b / 2, step))
    s, t = s.ravel(), t.ravel()
    c, sn = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    top = np.c_[cx + s * c - t * sn, cy + s * sn + t * c, SZ + h + rng.normal(0, 0.0005, s.size)]
    pts.append(top)
    # 옆면 한쪽(카메라가 조금 비스듬히 보는 쪽) — 윗면 띠 밖이라 yaw · 가운데에 섞이지 않아야 한다
    zs = np.arange(SZ + 0.002, SZ + h - 0.006, step)
    ss = np.arange(-a / 2, a / 2, step)
    S, Z = np.meshgrid(ss, zs)
    S, Z = S.ravel(), Z.ravel()
    pts.append(np.c_[cx + S * c - (b / 2) * sn, cy + S * sn + (b / 2) * c, Z])
    return np.vstack(pts)


@pytest.mark.parametrize('up,dx,dy,yaw', [('THICKNESS', 0.0, 0.0, 90.0), ('THICKNESS', 0.004, -0.003, 80.0),
                                          ('WIDTH', -0.002, 0.003, -85.0), ('LENGTH', 0.003, 0.002, 30.0)])
def test_measures_pose_centre_yaw(up, dx, dy, yaw):
    b = measure_on_spare(scene(up, SX + dx, SY + dy, yaw), SPOT, SIZE)
    assert b is not None and set(b) == KEYS
    json.dumps(b, allow_nan=False)
    assert b['up'] == up
    assert abs(b['x_m'] - (SX + dx)) < 0.001 and abs(b['y_m'] - (SY + dy)) < 0.001, b
    want = (yaw + 90.0) % 180.0 - 90.0
    assert abs((b['yaw_deg'] - want + 90.0) % 180.0 - 90.0) < 1.0, b
    assert -90.0 <= b['yaw_deg'] < 90.0
    assert b['top_z_m'] == round(SZ + POSE[up][2], 4) and b['base_z_m'] == round(SZ, 4)
    axes = {'THICKNESS': {'LENGTH', 'WIDTH'}, 'WIDTH': {'LENGTH', 'THICKNESS'}, 'LENGTH': {'WIDTH', 'THICKNESS'}}[up]
    assert set(b['gap_mm']) == axes and set(b['clear']) == axes
    assert all(v == OPEN_GAP_MM for v in b['gap_mm'].values()) and all(b['clear'].values())
    assert (b['overlap'], b['tilted'], b['confidence']) == ('none', False, 1.0)


def test_empty_support_and_too_few_points():
    """받침만 → None. 블록 점이 너무 적음(드문 격자) → None. 받침 가운데에서 먼 블록 → None. NaN 점은 무시."""
    g = np.arange(-0.04, 0.04, 0.001)
    X, Y = np.meshgrid(g, g)
    only = np.c_[SX + X.ravel(), SY + Y.ravel(), SZ + np.random.default_rng(1).normal(0, 0.001, X.size)]
    assert measure_on_spare(only, SPOT, SIZE) is None
    assert measure_on_spare(scene('THICKNESS', SX, SY, 90.0, step=0.008), SPOT, SIZE) is None
    assert measure_on_spare(scene('THICKNESS', SX + 0.12, SY, 90.0), SPOT, SIZE) is None
    assert measure_on_spare(np.zeros((0, 3)), SPOT, SIZE) is None
    pts = scene('THICKNESS', SX, SY, 90.0)
    pts = np.vstack([pts, np.full((10, 3), np.nan)])
    assert measure_on_spare(pts, SPOT, SIZE)['up'] == 'THICKNESS'


def test_ideal_block_matches_measure_fields():
    b = ideal_spare_block(SPOT, SIZE)
    assert set(b) == KEYS and b['up'] == 'THICKNESS' and (b['x_m'], b['y_m']) == (round(SX, 4), round(SY, 4))   # 0.1 mm 반올림
    assert b['yaw_deg'] == (SPOT['yaw_deg'] + 90.0) % 180.0 - 90.0 and b['top_z_m'] == round(SZ + T, 4)
    with pytest.raises(KeyError):
        measure_on_spare(np.zeros((0, 3)), {'x_m': 0.3}, SIZE)


# 받침 바로 위 카메라(관측 자세처럼 카메라 z 가 아래로): 카메라 축 = base (x, −y, −z), 받침 가운데에서 x −20 · y +10 mm 비킨 높이 0.40 m
DRAW_INTR = (600.0, 600.0, 320.0, 240.0)
DRAW_CAM = (SX - 0.020, SY + 0.010, 0.40)
DRAW_T = np.array([[1.0, 0.0, 0.0, DRAW_CAM[0]],
                   [0.0, -1.0, 0.0, DRAW_CAM[1]],
                   [0.0, 0.0, -1.0, DRAW_CAM[2]],
                   [0.0, 0.0, 0.0, 1.0]])


def _hand_pixel(x, y, z):
    """손 계산: 카메라 좌표 = (x − cx, −(y − cy), cz − z) → u = fx·X/Z + ppx, v = fy·Y/Z + ppy."""
    fx, fy, ppx, ppy = DRAW_INTR
    X, Y, Z = x - DRAW_CAM[0], -(y - DRAW_CAM[1]), DRAW_CAM[2] - z
    return fx * X / Z + ppx, fy * Y / Z + ppy


def test_project_to_image_matches_hand():
    """base 점 → 픽셀이 손 계산과 같다(받침 목표 윗면 가운데: u ≈ 359.3 · v ≈ 259.7)."""
    z = SZ + T
    uv = project_to_image([[SX, SY, z]], DRAW_INTR, DRAW_T)[0]
    assert np.allclose(uv, _hand_pixel(SX, SY, z), atol=1e-6)
    assert project_to_image([[SX, SY, 0.5]], DRAW_INTR, DRAW_T) is None      # 카메라보다 높은 점(카메라 뒤)은 None


def test_draw_spare_target_box_center_and_copy():
    """목표 자리 상자(빨강)의 가운데가 손 계산 픽셀 ±2 px · 긴 변(yaw 90 → base y)이 영상 세로 · 크기 그대로 · 입력은 안 고침 · 잰 블록 없어도 그린다."""
    pytest.importorskip('cv2')
    color = np.full((480, 640, 3), 90, np.uint8)
    target = ideal_spare_block(SPOT, SIZE)
    img = draw_spare(color, None, target, DRAW_INTR, DRAW_T, SIZE)
    assert img.shape == color.shape and img.dtype == color.dtype and (color == 90).all()
    vs, us = np.nonzero((img == TARGET_BGR).all(axis=2))
    assert len(us)
    cu, cv = (us.min() + us.max()) / 2, (vs.min() + vs.max()) / 2
    hu, hv = _hand_pixel(SX, SY, SZ + T)
    assert abs(cu - hu) <= 2 and abs(cv - hv) <= 2
    assert (vs.max() - vs.min()) > 2 * (us.max() - us.min())        # 74.45 × 24.8 mm 눕힘 — 긴 변이 세로


def test_draw_spare_measured_and_offset():
    """잰 블록이 있으면 초록 윤곽이 더 그려지고 목표 상자도 남는다. 둘 다 None 이면 그림은 원본과 같다(빈 그림 아님 — 바탕 그대로)."""
    pytest.importorskip('cv2')
    color = np.full((480, 640, 3), 90, np.uint8)
    target = ideal_spare_block(SPOT, SIZE)
    meas = {**target, 'x_m': SX + 0.0021, 'y_m': SY - 0.0008, 'yaw_deg': -87.0}
    img = draw_spare(color, meas, target, DRAW_INTR, DRAW_T, SIZE)
    assert (img == (0, 200, 0)).all(axis=2).any() and (img == TARGET_BGR).all(axis=2).any()
    assert (draw_spare(color, None, None, DRAW_INTR, DRAW_T, SIZE) == color).all()


def test_draw_spare_exact_fit_keeps_both_outlines():
    """잰 블록이 목표와 딱 맞아도 빨간 목표 테(굵게 먼저)와 초록 윤곽(가늘게 위)이 둘 다 보인다."""
    pytest.importorskip('cv2')
    color = np.full((480, 640, 3), 90, np.uint8)
    target = ideal_spare_block(SPOT, SIZE)
    img = draw_spare(color, dict(target), target, DRAW_INTR, DRAW_T, SIZE)
    assert (img == (0, 200, 0)).all(axis=2).any() and (img == TARGET_BGR).all(axis=2).any()


def test_spare_texts_dyaw_and_base_offset():
    """잰 블록 글자는 목표와의 yaw 차이(90° 경계에서 +90 · −90 이 번갈지 않게) · TARGET 아래 큰 글자는 dx · dy(mm) 한 줄
    (spare_offset_text 는 작업대 다시 재기 글자 placed_note 가 그대로 씀)."""
    target = ideal_spare_block(SPOT, SIZE)
    for yaw in (89.6, -89.6):                                  # 같은 자세(180° 대칭) — 목표 −90 과 차이는 ±0.4
        assert spare_label({**target, 'yaw_deg': yaw}, target) == 'SPARE FLAT dyaw +0'
    assert spare_label({**target, 'up': 'WIDTH', 'yaw_deg': target['yaw_deg'] + 3.2}, target) == 'SPARE EDGE dyaw +3'
    assert spare_label({**target, 'yaw_deg': 12.0}, None) == 'SPARE FLAT +12'
    meas = {**target, 'x_m': target['x_m'] + 0.0021, 'y_m': target['y_m'] - 0.0008}
    assert spare_offset_text(meas, target) == 'base dx +2.1 dy -0.8 mm'
    assert spare_offset_big(meas, target) == 'dx +2.1  dy -0.8 mm'


def test_draw_spare_offset_below_target_and_axes_above_box():
    """10/15 PL: 어긋난 거리는 TARGET 아래 큰 한 줄(왼쪽 위 작은 글자는 없앰), base x · y 화살표는 목표 상자 위(상자 밖 — 상자 안
    가운데는 보기 안 좋았다). 상자 안에는 흰 것(화살표 · 글자)이 없다."""
    cv2 = pytest.importorskip('cv2')
    color = np.full((480, 640, 3), 90, np.uint8)
    target = ideal_spare_block(SPOT, SIZE)
    meas = {**target, 'x_m': SX + 0.0021, 'y_m': SY - 0.0008, 'yaw_deg': target['yaw_deg'] + 3.0}
    img = draw_spare(color, meas, target, DRAW_INTR, DRAW_T, SIZE)
    white = (img == 255).all(axis=2)
    assert not white[:80, :120].any()                                       # 왼쪽 위(옛 글자 · 화살표 자리)는 비었다
    box = project_to_image(top_face_corners(target, SIZE), DRAW_INTR, DRAW_T)
    inside = np.zeros(white.shape, np.uint8)
    cv2.fillConvexPoly(inside, np.round(box).astype(np.int32), 1)
    assert not (white & inside.astype(bool)).any()                          # 상자 안은 비었다
    cu, top = int(box[:, 0].mean()), int(box[:, 1].min())
    assert white[:top - 26, cu - 60:cu + 60].any()                          # 잰 블록 글자(상자 위 ~18 px)보다 더 위에 화살표
    vs, us = np.nonzero(white & (np.arange(480)[:, None] > box[:, 1].max() + 20))   # TARGET 글자보다 아래
    (tw, _), _ = cv2.getTextSize(spare_offset_big(meas, target), cv2.FONT_HERSHEY_SIMPLEX, OFFSET_SCALE, 6)
    assert len(us) and us.max() - us.min() + 1 >= 0.8 * tw                  # 큰 한 줄이 거의 다 그려짐
    assert abs((us.min() + us.max()) / 2 - box[:, 0].mean()) <= 6           # 상자와 가로 가운데가 같다(글자 모양 치우침 몇 px)


def test_axes_spot_above_then_side_then_none():
    """축 화살표 자리: 상자 위(상자 밖, 가로 가운데) → 상자가 그림 위 끝에 붙어 위가 모자라면 오른쪽 → 왼쪽 → 다 막히면 None(왼쪽 위 구석)."""
    ext = (-10.0, -40.0, 30.0, 5.0)                                         # 묶음 상자(원점 기준) — +x 위 · +y 오른쪽 꼴
    g = lambda o: (o[0] + ext[0], o[1] + ext[1], o[0] + ext[2], o[1] + ext[3])
    mid = (300.0, 200.0, 400.0, 234.0)
    o = _axes_spot((640, 480), ext, mid, [mid, (290.0, 180.0, 410.0, 195.0)])   # 상자 + 그 위 글자
    assert g(o)[3] <= 180.0 - AXIS_GAP_PX + 1e-9 and abs((g(o)[0] + g(o)[2]) / 2 - 350.0) < 1e-9
    top = (300.0, 4.0, 400.0, 38.0)
    o = _axes_spot((640, 480), ext, top, [top])
    assert g(o)[0] >= 400.0 + AXIS_GAP_PX - 1e-9                            # 위가 모자라 오른쪽
    right = (560.0, 4.0, 636.0, 38.0)
    o = _axes_spot((640, 480), ext, right, [right])
    assert g(o)[2] <= 560.0 - AXIS_GAP_PX + 1e-9                            # 오른쪽도 모자라 왼쪽
    assert _axes_spot((640, 480), ext, mid, [(0.0, 0.0, 640.0, 480.0)]) is None


def test_base_axes_px_follows_camera_rotation():
    """화살표 방향 = base +x · +y 가 사진에서 가는 쪽. 시험 카메라(x 같음)면 → · ↑, 받침 관측 자세처럼 돌아간 카메라면 ↑ · ←(10/14 검토)."""
    target = ideal_spare_block(SPOT, SIZE)
    dx, dy = base_axes_px(target, DRAW_INTR, DRAW_T)
    assert np.allclose(dx, [1, 0], atol=1e-6) and np.allclose(dy, [0, -1], atol=1e-6)
    rot = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # 카메라 x = base −y · y = base −x · z 아래
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot, DRAW_CAM
    dx, dy = base_axes_px(target, DRAW_INTR, T)
    assert np.allclose(dx, [0, -1], atol=1e-6) and np.allclose(dy, [-1, 0], atol=1e-6)
