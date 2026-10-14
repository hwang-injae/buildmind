"""조립 자리에 방금 놓은 블록 하나 재기 — find_blocks 요청 {"area": "assembly", "design_id", "part_id"} (ROS 없는 계산, YOLO 없음).

W144(챌린지 A4 — 놓은 블록이 많이 비켰으면 다시 집어 놓기)의 측정 입력. 작업 관리자가 VERIFY 에서 방금 놓은 블록(위에 덮이기 전)을 묻는다.
받침 재기(spare_measure.measure_on_spare)와 같은 방식 — 설계 자리 둘레의 깊이 점 → 윗면 높이 · 윗면 점의 주축으로 yaw · 양 끝 백분위의 가운데.
받침과 다른 점: 둘레에 다른 블록이 있을 수 있어서, 블록 하나로 볼 수 없으면 좌표를 내지 않고 거절 코드를 낸다(잘못된 다시 집기를 막으려고).

거절 코드(reject): NO_PART(설계 자리에 그 높이의 윗면이 없음) · COVERED(설계 자리 위에 다른 물체가 덮음) · STANDING(세운 블록 — 윗면이
15 × 25 mm 라 지금은 재지 않음) · FEW_POINTS(윗면 점이 모자람) · TILTED(윗면이 5 mm 또는 5° 넘게 기욺) · NEIGHBOR_MERGED(같은 높이
이웃과 붙어 윗면이 블록보다 큼 — 경계가 안 보임) · SIZE_MISMATCH(윗면이 블록보다 4 mm 넘게 작음 — 가장자리가 가려지거나 깊이 구멍).
정확도는 실기로 아직 안 봤다(10/14) — 작업 관리자는 실기 흔들림을 확인한 뒤에 다시 집기에 쓴다(PL · 한석형).
단위: 입력 · 출력 좌표 m(base_link). 바깥 영향 없음.
"""
import math

import numpy as np
from scipy import ndimage

from d2_vision.block_finder import TILT_DEG, TILT_DEG_MIN_SHORT_MM, TILT_MM, TOP_AXES, wrap_deg
from d2_vision.spare_measure import EDGE_PCT, MIN_PTS_FLAT, UP_DIM

REACH_M = 0.015            # 설계 자리(위에서 본 상자) 둘레로 이만큼 넓혀 점을 본다 — 놓기 어긋남(몇 mm)과 이웃 붙음을 함께 보려고
TOP_TOL_M = 0.005          # 설계 윗면 높이에서 이만큼 안의 점만 이 블록 윗면 — 아래층(15 mm 아래) · 위 블록(15 mm 위)과 가른다
COVER_ABOVE_M = 0.008      # 설계 자리 안에서 윗면 + 이 값보다 높은 점(손가락 · 위 블록) 비율이
COVER_FRAC = 0.2           # 이 이상이면 COVERED
SIZE_TOL_M = 0.004         # 잰 윗면 두 변이 블록 치수와 이보다 많이 다르면 NEIGHBOR_MERGED(큼) · SIZE_MISMATCH(작음)
MIN_FRAC = 0.5             # 윗면 점이 받침 재기 최소(MIN_PTS_FLAT 비율)의 이 배보다 적으면 FEW_POINTS — 둘레 이웃에 가려 반쯤 보일 수 있어서
SEG_CELL_M = 0.002         # 윗면 점을 이 칸으로 나눠 이어진 덩어리만 이 블록으로 본다 — 틈(2 칸 이상)이 보이는 이웃은 떨어진 덩어리라 빠진다
EDGE_KEEP = (EDGE_PCT[1] - EDGE_PCT[0]) / 100.0   # 양 끝 백분위로 잰 폭 = 실제 폭 × 이 비율(고르게 퍼진 점) — 크기는 이걸로 나눠 되돌린다
REJECTS = ('NO_PART', 'COVERED', 'STANDING', 'FEW_POINTS', 'TILTED', 'NEIGHBOR_MERGED', 'SIZE_MISMATCH')


def up_of(rot):
    """블록 회전(base←블록 3x3) → 위를 향한 블록 축 이름(LENGTH · WIDTH · THICKNESS). rot 열 = 블록 축의 base 방향."""
    k = int(np.argmax(np.abs(np.asarray(rot, float)[2])))
    return ('LENGTH', 'WIDTH', 'THICKNESS')[k]


def _connected_to_center(sel, center_xy):
    """윗면 점 중 설계 가운데와 이어진 덩어리만 남긴다(SEG_CELL_M 칸, 8방향 이웃). 가운데 칸이 비면 가운데에서 가장 가까운 점의 덩어리.
    틈이 보이는 이웃 블록 윗면은 떨어진 덩어리라 빠지고, 붙은 이웃은 한 덩어리로 남아 뒤 크기 검사에서 NEIGHBOR_MERGED 가 된다."""
    ij = np.floor(sel[:, :2] / SEG_CELL_M).astype(int)
    lo = ij.min(axis=0)
    ij -= lo
    grid = np.zeros(ij.max(axis=0) + 1, bool)
    grid[ij[:, 0], ij[:, 1]] = True
    labels, _ = ndimage.label(grid, structure=np.ones((3, 3), bool))
    lab = labels[ij[:, 0], ij[:, 1]]
    k = int(np.argmin(np.hypot(sel[:, 0] - center_xy[0], sel[:, 1] - center_xy[1])))
    return sel[lab == lab[k]]


def measure_placed(pts, part, block_actual_m):
    """설계 자리에 놓인 블록 하나를 깊이 점으로 잰다.

    입력: pts = base 점 (N, 3) 또는 (H, W, 3) m(NaN 은 뺌) · part = 설계 자리 {part_id, center (x, y, z) m, rot 3x3}(motion_math.recipe_parts
          · BlockChecker.parts 와 같은 칸) · block_actual_m = [길이, 폭, 두께] m.
    출력: (블록 dict, None) — {part_id, x_m, y_m, top_z_m, yaw_deg, up, tilted, confidence, size_mm [두 변]} 또는 (None, (거절 코드, 설명)).
    순서: ① 세움이면 STANDING ② 설계 자리 둘레(REACH_M) 점 ③ 자리 안 높은 점 비율로 COVERED ④ 설계 윗면 ± TOP_TOL_M 점 — 없으면 NO_PART,
          가운데와 이어진 덩어리만(틈 보이는 이웃 뺌) — 모자라면 FEW_POINTS ⑤ 윗면 평면 기울기로 TILTED ⑥ 주축 yaw · 양 끝 백분위 가운데 ·
          두 변 길이(백분위 폭 ÷ EDGE_KEEP) → 크면 NEIGHBOR_MERGED, 작으면 SIZE_MISMATCH. 윗면 높이 = 윗면 점 가운데 값.
    바깥 영향 없음. 실패: part 칸이 없으면 KeyError."""
    L, W, T = (float(v) for v in block_actual_m)
    size = (L, W, T)
    rot = np.asarray(part['rot'], float)
    up = up_of(rot)
    if up == 'LENGTH':
        return None, ('STANDING', '세운 블록은 윗면이 15 × 25 mm 라 지금은 재지 않는다')
    cx, cy, cz = (float(v) for v in part['center'])
    half = np.array(size) / 2.0
    top_z = cz + float(np.abs(rot[2]) @ np.array(size)) / 2.0           # 설계 윗면 높이(base z)
    a, b = (size[UP_DIM[k]] for k in TOP_AXES[up])                     # 윗면 두 변(긴 · 짧은)
    p = np.asarray(pts, float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    local = (p - np.array([cx, cy, cz])) @ rot                          # 블록 좌표(rot^T · d)
    flat = [i for i in range(3) if ('LENGTH', 'WIDTH', 'THICKNESS')[i] != up]
    reach = np.all(np.abs(local[:, flat]) <= half[flat] + REACH_M, axis=1)
    inside = np.all(np.abs(local[:, flat]) <= half[flat], axis=1)
    above_in = inside & (p[:, 2] > top_z - TOP_TOL_M)
    n_cover = int(np.count_nonzero(inside & (p[:, 2] > top_z + COVER_ABOVE_M)))
    if n_cover and n_cover >= COVER_FRAC * max(int(np.count_nonzero(above_in)), 1):
        return None, ('COVERED', f'설계 자리 안 점 중 윗면 {COVER_ABOVE_M * 1000:.0f} mm 위가 {n_cover}개 — 손가락 · 위 블록이 덮음')
    sel = p[reach & (np.abs(p[:, 2] - top_z) <= TOP_TOL_M)]
    need = MIN_PTS_FLAT * a * b / (L * W)
    if len(sel) < max(need * 0.1, 3):
        return None, ('NO_PART', f'설계 윗면 높이 ±{TOP_TOL_M * 1000:.0f} mm 점이 {len(sel)}개 — 블록이 없거나 다른 높이')
    sel = _connected_to_center(sel, (cx, cy))
    if len(sel) < max(need * MIN_FRAC, 3):
        return None, ('FEW_POINTS', f'윗면 점 {len(sel)}개 < {need * MIN_FRAC:.0f}개')
    A = np.c_[sel[:, 0], sel[:, 1], np.ones(len(sel))]
    coef = np.linalg.lstsq(A, sel[:, 2], rcond=None)[0]
    xy = sel[:, :2]
    mean = xy.mean(axis=0)
    w, vec = np.linalg.eigh(np.cov((xy - mean).T))
    u = vec[:, int(np.argmax(w))]
    v = np.array([-u[1], u[0]])
    lo_u, hi_u = np.percentile((xy - mean) @ u, EDGE_PCT)
    lo_v, hi_v = np.percentile((xy - mean) @ v, EDGE_PCT)
    span_u, span_v = (hi_u - lo_u) / EDGE_KEEP, (hi_v - lo_v) / EDGE_KEEP   # 백분위 폭 → 실제 폭
    tilt_deg = math.degrees(math.atan(math.hypot(coef[0], coef[1])))
    tilt_mm = abs(coef[0] * u[0] + coef[1] * u[1]) * span_u              # 긴 축 양 끝 높이 차
    if tilt_mm >= TILT_MM / 1000.0 or (b * 1000.0 >= TILT_DEG_MIN_SHORT_MM and tilt_deg >= TILT_DEG):
        return None, ('TILTED', f'윗면 양 끝 높이 차 {tilt_mm * 1000:.1f} mm · 기울기 {tilt_deg:.1f}°')
    if span_u > a + SIZE_TOL_M or span_v > b + SIZE_TOL_M:
        return None, ('NEIGHBOR_MERGED', f'윗면 {span_u * 1000:.0f} × {span_v * 1000:.0f} mm > 블록 {a * 1000:.0f} × {b * 1000:.0f} mm — 같은 높이 이웃과 붙어 경계가 안 보임')
    if span_u < a - SIZE_TOL_M or span_v < b - SIZE_TOL_M:
        return None, ('SIZE_MISMATCH', f'윗면 {span_u * 1000:.0f} × {span_v * 1000:.0f} mm < 블록 {a * 1000:.0f} × {b * 1000:.0f} mm — 가장자리가 안 보임')
    c = mean + u * (lo_u + hi_u) / 2 + v * (lo_v + hi_v) / 2
    yaw = round(wrap_deg(math.degrees(math.atan2(u[1], u[0]))), 1)
    return {
        'part_id': part['part_id'],
        'x_m': round(float(c[0]), 4),
        'y_m': round(float(c[1]), 4),
        'top_z_m': round(float(np.median(sel[:, 2])), 4),                # 윗면 점의 가운데 값 — 98 % 백분위는 깊이 잡음만큼 높게 나온다
        'yaw_deg': yaw if yaw < 90.0 else -90.0,
        'up': up,
        'tilted': False,
        'confidence': 1.0,
        'size_mm': [round(span_u * 1000.0, 1), round(span_v * 1000.0, 1)],
    }, None
