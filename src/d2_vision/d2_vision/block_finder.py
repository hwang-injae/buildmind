"""흩어진 블록 찾기 BlockFinder — 마스크 → 블록마다 자세 · yaw · 중심 · 틈 · 겹침 · 기울기 (ROS 없는 계산, W086, SDD §6.11 ②).

노드(wrist_block 의 /d2/vision/find_blocks)가 공급 관측 자세(observe_supply)에서 찍은 컬러 · 깊이로 부른다. 계산만 한다 — 로봇 · 메시지 · 파일에 손대지 않는다.

두 단계로 나눈다(E-47: 본 방법 = YOLO seg, 예비 = 엣지 + 깊이):
  ① 마스크 만들기 — YOLO seg 결과(블록마다 보이는 윤곽 마스크)를 그대로 받거나, 모델이 없으면 masks_from_edges()(예비)로 만든다.
  ② 뒷단(공용) blocks_from_masks() — 마스크마다 깊이 → base 점 → 윗면 평면 → 자세 up · yaw · 중심 · 틈(gap_mm · clear) · overlap · tilted.
     YOLO 마스크든 예비 마스크든 같은 뒷단을 쓴다.

출력은 IRD `/d2/vision/find_blocks` 응답의 blocks 원소 그대로(dict, JSON 직렬화 가능, NaN 없음), 윗면 높이 순:
  {"x_m","y_m","top_z_m","yaw_deg","up","clear":{축:bool},"gap_mm":{축:mm},"overlap","tilted","confidence"}
  - x_m · y_m · top_z_m: base_link (m), 윗면 가운데. 기울어진 블록(tilted)의 top_z_m 만 윗면에서 가장 높은 점(평면 98 % 높이) —
    작업 관리자가 이 값을 윗면으로 장애물 상자를 만들어서(E-68 '가장 높은 점까지 닿는 상자', 10/11 박진용). x_m · y_m 은 그대로 가운데.
  - up: 작업면 위 높이를 먼저(15 · 25 · 75 + 포개짐 층, ±5 mm) 보고 그 후보 안에서만 윗면 모양으로 가린다. 어긋나면 confidence 절반(10/8 PL).
    다른 블록 위에 얹힌 블록은 '내 높이 − 아래 블록 높이'를 내 두께로 다시 본다. 영상 테두리 · 영역 경계에 잘린 윤곽은 under · confidence ≤ 0.2.
  - yaw_deg: 윗면의 긴 변이 base x 와 평행하면 0, 반시계 +, −90 ≤ yaw < 90 (motion_math.wrap_half 와 같은 경계).
    세움(up LENGTH)은 윗면 25 mm 변(WIDTH) 기준(10/8 E-62).
  - gap_mm · clear 의 키 = 블록의 수평인 두 축(눕힘 LENGTH · WIDTH, 옆세움 LENGTH · THICKNESS, 세움 WIDTH · THICKNESS — E-55 ③).
    gap_mm.<축> = 그 축 방향으로 손가락이 닫힐 때(블록 양끝 바깥으로 손가락이 내려갈 때) 양쪽 틈 중 좁은 쪽(mm). GAP_SEARCH_MM 안에 막는 게 없으면 OPEN_GAP_MM(99).
    clear.<축> = gap_mm ≥ find.min_gap_mm.
  - overlap: none · top(다른 블록 위에 올라앉음 · 걸침) · under(다른 블록에 덮임 — 보이는 면이 작거나 위에 걸친 블록이 맞닿음,
    또는 모양이 블록 하나와 안 맞는 덩어리 — 어느 쪽이든 집지 않고 틈 · 장애물로만 쓴다).
  - tilted: 윗면 평면의 양 끝 높이 차 ≥ TILT_MM(5 mm, E-43) **또는** 윗면 기울기 ≥ TILT_DEG(5°, 10/14 PL B1 — 윗면이 넓은 눕힌 면에서만,
    TILT_DEG_MIN_SHORT_MM). mm 기준만이면 옆으로 5 ~ 11° 기운 눕힌 블록(25 mm 폭에서 5 mm 미만)을 못 걸렀다.
  - tilted 가 true 인 블록에만 두 칸을 더한다(10/13): center_top_z_m = 윗면 가운데 평면 높이(m) · normal = 윗면 단위 법선 [nx, ny, nz]
    (위쪽, 평면 맞춤에서). 기울지 않은 블록은 위 칸 그대로.
단위: 이 파일 안 좌표는 m(base_link), 치수 · 틈 · 높이 비교는 mm. 높이 지도는 block_checker.height_map(스캔 추론기와 같은 함수)을 쓴다.
cv2 는 예비 마스크(masks_from_edges) · 그림(draw_found · draw_spare · draw_workbench)에서만 쓴다 — 뒷단은 numpy 만 써서 CI(cv2 없음)에서도 시험한다.
"""
import itertools
import math

import numpy as np

from d2_vision.block_checker import depth_to_base_points, height_map

UPS = ('THICKNESS', 'WIDTH', 'LENGTH')
# up → (윗면 긴 변 축, 윗면 짧은 변 축). gap_mm · clear 의 키가 이 두 축이다(IRD find_blocks, E-55 ③)
TOP_AXES = {'THICKNESS': ('LENGTH', 'WIDTH'), 'WIDTH': ('LENGTH', 'THICKNESS'), 'LENGTH': ('WIDTH', 'THICKNESS')}
DIM = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}       # size_m 안 순서 (robot.yaml block_actual_m = [길이, 폭, 두께])

Z_MIN_M, Z_MAX_M = 0.1, 1.0   # 깊이 유효 범위 — block_checker.depth_to_base_points 기본값과 같게(픽셀 ↔ 점 순서를 맞추려고)
TILT_MM = 5.0             # 양 끝 높이 차 이상이면 tilted (IRD · E-43)
TILT_DEG = 5.0            # 윗면 기울기 각이 이 이상이어도 tilted — 집기 노드 보통 집기 한계(motion_math.LEVEL_TOL_DEG 5°)와 같게(10/14 PL B1)
TILT_DEG_MIN_SHORT_MM = 20.0   # 각도 기준은 윗면 짧은 변이 이 이상(눕힌 75 × 25 면)일 때만 — 세움 · 옆세움의 15 mm 윗면은 깊이 잡음 1 mm 로도 각이 4° 튄다
TABLE_WINDOW_MM = 6.0     # 작업면 높이 다시 재기: robot.yaml table_z 근처 ±이 안의 점 중앙값 (10/8 실측 장면이 yaml 값과 1.5 mm 다름)
TOP_MIN_MM = 4.0          # 작업면 위 이보다 낮은 점은 윗면 후보에서 뺀다(깊이 잡음 ±2~3 mm)
MAX_SLOPE = 0.6           # 이웃 픽셀 사이 높이 기울기(dz/dxy)가 이보다 크면 옆면 · 가장자리 튐 — 윗면에서 뺀다(31°보다 가파름)
EDGE_PX = 3               # 평면 맞춤 때 마스크 가장자리를 이만큼 깎는다 — 경계 깊이 튐(SDD §6.11 ② "가장자리 몇 픽셀은 버림")
PLANE_RESID_MM = 4.0      # 평면에서 이보다 먼 점은 윗면이 아니다(옆면 · 다른 블록)
MIN_TOP_PTS = 30          # 윗면 점이 이보다 적으면 그 마스크는 블록으로 내지 않는다(깊이 구멍 · 너무 가려짐)
SHRINK_MM = 4.5           # 기울기 · 가장자리 거르기로 윗면 크기가 이만큼 작게 재진다 — 치수 비교 때 더해 준다
                          # (W114-v2 실측: 떨어진 눕힘 블록 윗면이 70 x 20 mm 로 재짐 → 74.5 x 24.8 과 맞춤. 합성 영상은 1~2 mm 라 조금 크게 나온다)
HEIGHT_TOL_MM = 5.0       # 자세 판정 ① 높이: 작업면 위 높이가 후보 높이(위 치수 + 받침 층) ± 이 안이면 후보 (10/8 PL — 높이 먼저)
SIGMA_H_MM = 4.0          # 높이 후보 안에서 같은 모양 점수일 때 높이가 더 가까운 쪽을 고르는 데만 쓴다
STACK_PENALTY = 2.0       # 자세 판정 ② 모양 점수에 받침 블록 하나마다 더하는 값 — 비슷하면 덜 쌓였다고 본다
SHAPE_MISMATCH = 4.0      # 고른 자세의 모양 점수가 이보다 크면(≈ 2σ) '높이와 모양이 어긋남' → confidence 절반(자세는 높이를 따른다)
FAT_PER_MM = (0.15, 0.10) # 높은 블록일수록 깊이가 윗면을 넓게 잰다(W114-v2 실측: 세운 블록 윗면 25x15 → 40x28 mm) —
                          # 기대보다 큰 쪽 허용 폭을 작업면 위 FAT_FROM_MM 넘는 높이 1 mm 마다 (긴 변, 짧은 변) 이만큼 넓힌다
FAT_FROM_MM = 30.0        # 이 높이까지는 넓히지 않는다 — 눕힘 · 옆세움 · 두 층(≤ 30 mm)은 실측에서 윗면이 오히려 작게 재짐
MAX_COST = 9.0            # 모양 점수가 이보다 크면(≈ 3σ) 블록 하나 모양이 아니다(붙은 두 블록 · 잘못 나뉜 마스크) → 집지 않게 under
BORDER_PX = 3             # 마스크가 영상 테두리 이 픽셀 안에 닿으면 잘린 윤곽 — 자세 · yaw 를 믿지 않는다(10/8 PL)
CLIP_RING_PX = 3          # 마스크를 이만큼 넓힌 테두리에 흩뿌릴 영역 밖 블록 높이 픽셀이 있으면 영역 경계에서 잘린 윤곽
CLIP_H_FRAC = 0.5         # 그 '블록 높이' = 내 높이의 이 비율 이상 — 내 가장자리 깊이 번짐(작업면 위 5~8 mm)은 잘림으로 세지 않는다
CLIP_OUT_FRAC = 0.02      # 마스크 픽셀 중 영역 밖 비율이 이보다 크면 잘린 윤곽(YOLO 마스크는 영역을 모른다)
CLIP_CONF = 0.2           # 잘린 윤곽의 confidence 상한 — under 로 내보내 집지 않게 한다
UNDER_AREA_RATIO = 0.55   # 보이는 면적 ÷ 기대 윗면 면적이 이보다 작으면 under(덮임)
CONTACT_PX = 5            # 두 마스크가 이 픽셀(약 4 mm) 안이면 맞닿음
UNDER_TOL_MM = 4.0        # 맞닿은 더 높은 블록의 밑면이 내 윗면 − 이 값보다 높으면 '나를 덮음'
HALO_PX = 6               # 틈 셀 때 내 마스크를 이만큼(약 5 mm) 넓혀 내 블록의 가장자리 깊이 번짐을 장애물로 세지 않는다
FOOT_MARGIN_MM = 3.0      # 틈 셀 때 내 블록 자리(중심 ± 치수/2)를 이만큼 넓혀 장애물에서 뺀다 — 마스크가 블록보다 짧아도
                          # (YOLO 실측: 길이 71 mm 로 재진 블록) 내 윗면 끝이 장애물로 세지지 않게
OTHER_DILATE_PX = 2       # 다른 블록 마스크를 이만큼 넓혀 장애물 후보로 — 마스크가 블록보다 조금 작아도 옆 블록 가장자리를 놓치지 않게
NONMASK_MIN_MM = 10.0     # 어느 마스크에도 없는 점(못 찾은 블록일 수 있음)은 작업면 위 이보다 높고 완만할 때만 장애물 — 내 블록 둘레 깊이 번짐
                          # (W114 s04_002 실측: 면 밖 0~15 mm 에 높이 5~13 mm 비탈)과 종이 주름(~8 mm)을 빼려고. 블록 윗면은 실측 ≥ 10 mm
STEEP_ABOVE_MM = 3.0      # 마스크 밖의 가파른 점도 내 윗면 + 이 값보다 높으면 장애물 — 옆에 선 · 포갠 높은 블록의 옆면(윗면은 작아 완만한 점이 적다)
GAP_CELL_M = 0.002        # 틈 계산 높이 지도 셀 — 8 mm 틈을 ±1.4 mm 로 재려고 5 mm 대신 2 mm
GAP_SEARCH_MM = 60.0      # 손가락 자리 둘레를 이 거리까지 본다(22.2 mm 기준의 2.7배)
OPEN_GAP_MM = 99.0        # 찾는 거리 안에 막는 게 없으면 내는 '넉넉함' 틈 — 받침 재기(spare_measure)와 같은 값 한 곳(10/14 PL B3, 박진용 실기 값)
STRIP_MARGIN_MM = 2.0     # 손가락 폭(finger.width_m) 양옆 여유 — 손가락이 지나갈 띠의 반폭 = 폭/2 + 이 값
OBST_MIN_MM = 5.0         # 작업면 위 이보다 낮은 것(테이프 · 잡음)은 장애물이 아니다
FINGER_Z_MARGIN_MM = 5.0  # 손가락 끝 높이(윗면 − grasp_depth)보다 이만큼 아래까지 장애물로 본다(세운 블록은 더 깊게 문다 — robot.yaml 공급 칸)


def finder_cfg(robot):
    """robot.yaml 을 읽은 dict → BlockFinder 설정 dict. 파일 읽기는 부르는 쪽(노드 · 시험)이 한다.

    입력: robot = robot.yaml dict (block_actual_m 또는 block_size_m · table_z_m · find.min_gap_mm · grasp_depth_m · finger.width_m ·
    supply_area_m {x_m: [최소, 최대], y_m: [최소, 최대]} — 흩뿌린 공급 영역(빨간 테이프) 안쪽 직사각형 m, 10/11 박진용 교시 · 민범진 확인).
    출력: {'size_m': (길이, 폭, 두께) m, 'table_z_m', 'min_gap_mm', 'grasp_depth_m', 'finger_width_m', 'area_m': ((x0,x1),(y0,y1)) m, 'clip_margin_m': 잘린 윤곽 판정 영역 여유 m(손가락 여유, 키가 없으면 0)}.
    실패: 키가 없으면 KeyError, supply_area_m 의 x_m · y_m 이 두 수가 아니거나 최소 ≥ 최대면 ValueError (설정 오류 — 노드가 시작할 때 드러나게).
    """
    size = robot.get('block_actual_m') or robot['block_size_m']    # 실측 치수가 있으면 그것(쌓는 높이 · 장면 상자와 같은 값)
    return {
        'size_m': tuple(float(v) for v in size),
        'table_z_m': float(robot['table_z_m']),
        'min_gap_mm': float(robot['find']['min_gap_mm']),
        'grasp_depth_m': float(robot['grasp_depth_m']),
        'finger_width_m': float(robot['finger']['width_m']),
        'area_m': _area_rect(robot['supply_area_m']),
        # 잘린 윤곽 판정(_clipped)만 영역을 손가락 여유(가장 넓게 연 폭/2 + 손가락 두께)만큼 넓혀 본다 — 10/13 004 실기: 윤곽이 다 보이는
        # 블록이 귀퉁이 3 mm 가 테이프 안쪽 직사각형 밖이라 '잘림 → 덮임'으로 빠져 공급 대기. 높이 지도(height_grid.supply_map_area)와 같은 여유
        'clip_margin_m': max(float(v) for v in robot['grasp_open_pick_m'].values()) / 2 + float(robot['finger']['thickness_m'])
        if 'grasp_open_pick_m' in robot and 'finger' in robot and 'thickness_m' in robot['finger'] else 0.0,
    }


def _area_rect(area):
    """robot.yaml supply_area_m {x_m: [최소, 최대], y_m: [최소, 최대]} (m) → ((x0, x1), (y0, y1)).
    dict 가 아니거나(옛 꼭짓점 목록 꼴 — #126 첫 커밋) x_m · y_m 이 없거나 두 수가 아니거나 최소 ≥ 최대면 ValueError."""
    if not isinstance(area, dict) or not {'x_m', 'y_m'} <= area.keys():
        raise ValueError(f'supply_area_m 은 {{x_m: [최소, 최대], y_m: [최소, 최대]}} 이어야 한다: {area!r}')
    out = []
    for k in ('x_m', 'y_m'):
        try:
            lo, hi = (float(v) for v in area[k])
        except (TypeError, ValueError):                 # 두 수가 아님(개수 · 글자)
            lo = hi = None
        if lo is None or not lo < hi:
            raise ValueError(f'supply_area_m.{k} 는 [최소, 최대] 이고 최소 < 최대여야 한다: {area[k]!r}')
        out.append((lo, hi))
    return tuple(out)


def wrap_deg(a):
    """각도(도)를 −90 이상 90 미만으로. 블록은 180° 대칭 — motion_math.wrap_half 와 같은 식(d2_motion 을 import 하지 않으려고 여기 둔다)."""
    return (a + 90.0) % 180.0 - 90.0


# ---------------- 픽셀 도구 (numpy 만) ----------------
def _dilate(m, r):
    """bool 영상을 (2r+1) 정사각형으로 넓힌다. cv2 없이 이웃 밀기로."""
    out = m.copy()
    for _ in range(int(r)):
        o = out.copy()
        o[1:, :] |= out[:-1, :]
        o[:-1, :] |= out[1:, :]
        out = o.copy()
        out[:, 1:] |= o[:, :-1]
        out[:, :-1] |= o[:, 1:]
    return out


def _erode(m, r):
    """bool 영상을 (2r+1) 정사각형으로 깎는다."""
    return ~_dilate(~m, r)


def _bbox(m, pad, shape):
    """마스크를 감싸는 (v0, v1, u0, u1) 창, 사방 pad 픽셀 넓힘(영상 안으로 자름). 빈 마스크면 None."""
    vs, us = np.nonzero(m)
    if len(vs) == 0:
        return None
    return (max(vs.min() - pad, 0), min(vs.max() + pad + 1, shape[0]),
            max(us.min() - pad, 0), min(us.max() + pad + 1, shape[1]))


def base_image(depth_m, intr, T_base2cam):
    """깊이 영상 → 픽셀마다 base 점 (H, W, 3) m. 깊이 없는 픽셀은 NaN. block_checker.depth_to_base_points(stride 1)를 픽셀 자리로 되돌려 놓는다."""
    depth_m = np.asarray(depth_m, float)
    ok = (depth_m > Z_MIN_M) & (depth_m < Z_MAX_M)
    P = np.full(depth_m.shape + (3,), np.nan)
    P[ok] = depth_to_base_points(depth_m, intr, np.asarray(T_base2cam, float), stride=1, z_min_m=Z_MIN_M, z_max_m=Z_MAX_M)
    return P


def table_height(P, cfg):
    """작업면 높이(m)를 영상에서 다시 잰다: 흩뿌릴 영역 안, robot.yaml table_z_m ±TABLE_WINDOW_MM 안 점의 중앙값.
    그런 점이 영역 점의 20 % 미만이면(블록이 거의 덮음 · 깊이 없음) robot.yaml 값을 그대로 쓴다."""
    z0 = cfg['table_z_m']
    inside = _in_area(P, cfg) & np.isfinite(P[..., 2])
    z = P[..., 2][inside]
    near = z[np.abs(z - z0) < TABLE_WINDOW_MM / 1000.0]
    if len(z) == 0 or len(near) < 0.2 * len(z):
        return z0
    return float(np.median(near))


def _in_area(P, cfg):
    """픽셀의 base (x, y)가 흩뿌릴 영역 안인가 (H, W) bool. area_m 이 None 이면 전부 참."""
    area = cfg.get('area_m')
    if area is None:
        return np.ones(P.shape[:2], bool)
    (x0, x1), (y0, y1) = area
    with np.errstate(invalid='ignore'):
        return (P[..., 0] >= x0) & (P[..., 0] <= x1) & (P[..., 1] >= y0) & (P[..., 1] <= y1)


def slope_image(P):
    """픽셀마다 이웃(±1 픽셀) 사이 높이 기울기 |dz| / |dxy| 의 큰 값. 옆면 · 경계는 크고 윗면은 작다. 계산 못 하면 inf."""
    z, x, y = P[..., 2], P[..., 0], P[..., 1]
    out = np.full(z.shape, np.inf)
    with np.errstate(invalid='ignore', divide='ignore'):
        su = np.abs(z[:, 2:] - z[:, :-2]) / np.hypot(x[:, 2:] - x[:, :-2], y[:, 2:] - y[:, :-2])
        sv = np.abs(z[2:, :] - z[:-2, :]) / np.hypot(x[2:, :] - x[:-2, :], y[2:, :] - y[:-2, :])
        s = np.fmax(su[1:-1, :], sv[:, 1:-1])
    s[~np.isfinite(s)] = np.inf
    out[1:-1, 1:-1] = s
    return out


def _levels(size_mm):
    """블록이 놓일 수 있는 받침: [(높이 mm, 받침 블록 수)] — 작업면(0, 0) + 블록 치수 1~3개를 더한 값(포개짐). 같은 높이는 적은 수로."""
    out = {0.0: 0}
    for n in (1, 2, 3):
        for c in itertools.combinations_with_replacement(size_mm, n):
            out.setdefault(round(sum(c), 1), n)
    return sorted(out.items())


# ---------------- 뒷단 (공용) ----------------
def _measure(mask, P, slope, table_z, depth_m, intr, cfg, score):
    """마스크 하나 → 윗면 평면 · 자세 · 중심 · yaw 등(내부 dict). 윗면 점이 MIN_TOP_PTS 보다 적으면 None.

    순서: 가장자리 깎은 마스크 안 '완만한 점'으로 평면 z = a·x + b·y + c 맞춤(먼 점 빼고 한 번 더) → 평면에 붙은 점 = 윗면
    → 윗면 점 PCA 로 긴 변 방향 · 길이 · 폭 · 가운데 → 자세 판정(_classify)."""
    h_all = (P[..., 2] - table_z) * 1000.0
    with np.errstate(invalid='ignore'):
        usable = np.isfinite(h_all) & (h_all > TOP_MIN_MM) & (slope < MAX_SLOPE)
    sel = _erode(mask, EDGE_PX) & usable
    if sel.sum() < MIN_TOP_PTS:                  # 작은 마스크(세움 윗면 · 가려진 조각)는 깎지 않고 본다
        sel = mask & usable
    if sel.sum() < MIN_TOP_PTS:
        return None
    pts = P[sel]
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    coef = np.linalg.lstsq(A, pts[:, 2], rcond=None)[0]
    keep = np.abs(A @ coef - pts[:, 2]) < PLANE_RESID_MM / 1000.0
    if keep.sum() >= MIN_TOP_PTS:
        coef = np.linalg.lstsq(A[keep], pts[keep, 2], rcond=None)[0]
    # 윗면 = 가장자리를 깎지 않은 마스크 중 평면에 붙은 완만한 점
    cand = mask & usable
    with np.errstate(invalid='ignore'):
        top = cand & (np.abs(P[..., 0] * coef[0] + P[..., 1] * coef[1] + coef[2] - P[..., 2]) < PLANE_RESID_MM / 1000.0)
    if top.sum() < MIN_TOP_PTS:
        return None
    xy = P[top][:, :2]
    mean = xy.mean(axis=0)
    w, vec = np.linalg.eigh(np.cov((xy - mean).T))
    u = vec[:, np.argmax(w)]
    v = np.array([-u[1], u[0]])
    pu, pv = (xy - mean) @ u, (xy - mean) @ v
    lo_u, hi_u = np.percentile(pu, [1, 99])
    lo_v, hi_v = np.percentile(pv, [1, 99])
    center = mean + u * (lo_u + hi_u) / 2 + v * (lo_v + hi_v) / 2
    long_mm = (hi_u - lo_u) * 1000.0 + SHRINK_MM
    short_mm = (hi_v - lo_v) * 1000.0 + SHRINK_MM
    zp = xy @ coef[:2] + coef[2]
    z_lo, z_hi = np.percentile(zp, [2, 98])
    z_c = float(center @ coef[:2] + coef[2])
    tilt_mm = float((z_hi - z_lo) * 1000.0)
    tilt_deg = math.degrees(math.atan(math.hypot(coef[0], coef[1])))   # 평면 z = a·x + b·y + c 의 기울기 각
    tilted = tilt_mm >= TILT_MM or (short_mm >= TILT_DEG_MIN_SHORT_MM and tilt_deg >= TILT_DEG)
    h_c = (z_c - table_z) * 1000.0
    h_ref = (z_lo - table_z) * 1000.0 if tilted else h_c   # 기울어진 블록은 낮은 끝(바닥에 닿은 쪽)으로 받침을 본다
    up, cost, base_mm, mismatch = _classify(long_mm, short_mm, h_ref, cfg)
    # 보이는 면적: 마스크 픽셀 수 × 윗면 깊이에서 픽셀 하나의 넓이 — 깊이 구멍이 있어도 마스크로 센다
    fx, fy = intr[0], intr[1]
    d_med = float(np.median(depth_m[top]))
    vis_area = float(mask.sum()) * (d_med / fx) * (d_med / fy) * 1e6
    base_conf = score * min(1.0, top.sum() / max(sel.sum(), 1))
    return {
        'mask': mask, 'top': top, 'coef': coef, 'center': center, 'u': u, 'v': v,
        'z_c': z_c, 'z_hi': float(z_hi), 'h_c': h_c, 'h_ref': h_ref, 'tilted': tilted, 'tilt_mm': tilt_mm, 'tilt_deg': tilt_deg, 'up': up, 'base_mm': base_mm,
        'long_mm': long_mm, 'short_mm': short_mm, 'vis_area': vis_area, 'cost': cost, 'mismatch': mismatch,
        'base_conf': base_conf, 'under': False, 'top_of': False, 'on': [],
    }


def _shape_cost(up, long_mm, short_mm, h_mm, size_mm):
    """윗면 모양(긴 변 · 짧은 변 mm)이 자세 up 의 윗면(75×25 · 75×15 · 25×15)과 얼마나 다른가(제곱 합, 0 = 같음).
    가려진 블록은 보이는 면이 작으니 기대보다 작은 쪽은 너그럽게(σ 20 · 8 mm), 큰 쪽은 엄하게(σ 8 · 5 mm + 높이 × FAT_PER_MM) 본다."""
    e_long, e_short = (size_mm[DIM[a]] for a in TOP_AXES[up])
    hh = max(h_mm - FAT_FROM_MM, 0.0)
    c_l = ((long_mm - e_long) / (8.0 + FAT_PER_MM[0] * hh if long_mm > e_long else 20.0)) ** 2
    c_s = ((short_mm - e_short) / (5.0 + FAT_PER_MM[1] * hh if short_mm > e_short else 8.0)) ** 2
    return c_l + c_s


def _classify(long_mm, short_mm, h_ref_mm, cfg):
    """작업면 위 높이(mm)를 먼저, 윗면 모양은 보조로 → (up, 모양 점수, 받침 높이 mm, 어긋남 bool). (10/8 PL 규칙)

    ① 높이: 후보 = 위 치수(15 · 25 · 75) + 받침 층(0 · 블록 1~3개 합 — 15+15, 15+25 …). 높이가 ±HEIGHT_TOL_MM 안인 후보만 남긴다.
       맞는 후보가 없으면 가장 가까운 높이 하나를 쓰고 어긋남으로 표시한다.
    ② 모양: 남은 후보 중 모양 점수(+ 받침 블록 수 × STACK_PENALTY, 같으면 높이가 가까운 쪽)가 가장 작은 것.
    ③ 고른 자세의 모양 점수 > SHAPE_MISMATCH 면 어긋남(부르는 쪽이 confidence 를 낮춘다) — 자세는 높이를 따른 그대로."""
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    cands = [(abs(h_ref_mm - size_mm[DIM[up]] - L), up, L, n) for up in UPS for L, n in _levels(size_mm)]
    near = [c for c in cands if c[0] <= HEIGHT_TOL_MM]
    mismatch = not near
    if not near:
        near = [min(cands)]

    def key(c):
        return _shape_cost(c[1], long_mm, short_mm, h_ref_mm, size_mm) + STACK_PENALTY * c[3] + 0.1 * (c[0] / SIGMA_H_MM) ** 2

    dh, up, lvl, _ = min(near, key=key)
    shape = _shape_cost(up, long_mm, short_mm, h_ref_mm, size_mm)
    mismatch = mismatch or shape > SHAPE_MISMATCH
    return up, shape + (dh / SIGMA_H_MM) ** 2, lvl, mismatch


def _refine_top(blk, cfg):
    """다른 블록 위에 얹힌 블록(맞닿은 아래 블록이 있음, 기울지 않음)의 자세를 '내 높이 − 아래 블록 높이' = 내 두께로 다시 정한다(10/8 PL).
    두께가 15 · 25 · 75 중 ±HEIGHT_TOL_MM 안이고 모양도 맞는 자세가 있으면 그것(여럿이면 모양으로), 없으면 그대로 둔다. 입력 dict 를 고침."""
    if blk['tilted'] or not blk['on']:
        return
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    # 맞닿은 아래 블록이 여럿이면 높은 것부터 — 맞닿았다고 다 받침은 아니라서(보이지 않는 받침 위에 얹혀 낮은 블록과 옆으로 닿을 수 있음)
    # 두께가 맞고 모양도 맞는(SHAPE_MISMATCH 이하) 첫 받침만 쓴다. 없으면 높이 먼저 고른 자세 그대로.
    for a in sorted(blk['on'], key=lambda k: -k['h_c']):
        own = blk['h_c'] - a['h_c']
        ups = [(_shape_cost(up, blk['long_mm'], blk['short_mm'], blk['h_c'], size_mm), up) for up in UPS
               if abs(own - size_mm[DIM[up]]) <= HEIGHT_TOL_MM]
        ups = [c for c in ups if c[0] <= SHAPE_MISMATCH]
        if ups:
            shape, up = min(ups)
            blk.update(up=up, base_mm=a['h_c'], cost=shape + ((own - size_mm[DIM[up]]) / SIGMA_H_MM) ** 2, mismatch=False)
            return


def _clipped(mask, P, h_mm, h_c, cfg):
    """윤곽이 잘렸나(bool): 영상 테두리 BORDER_PX 안에 닿음 · 마스크 픽셀이 흩뿌릴 영역(손가락 여유 clip_margin_m 만큼 넓힌 것) 밖(CLIP_OUT_FRAC 넘게) ·
    마스크 바로 바깥(CLIP_RING_PX)에 영역 밖이면서 내 높이 h_c(mm)의 CLIP_H_FRAC 이상인 픽셀이 있음(블록이 영역 밖으로 이어짐 —
    예비 마스크는 영역 안에서만 만들어져 경계에서 잘려 나온다)."""
    b = BORDER_PX
    if mask[:b].any() or mask[-b:].any() or mask[:, :b].any() or mask[:, -b:].any():
        return True
    if cfg.get('area_m') is None:
        return False
    win = _bbox(mask, CLIP_RING_PX + 1, mask.shape)
    v0, v1, u0, u1 = win
    m = mask[v0:v1, u0:u1]
    Pw = P[v0:v1, u0:u1]
    mg = float(cfg.get('clip_margin_m') or 0.0)
    (x0, x1), (y0, y1) = cfg['area_m']
    inside = _in_area(Pw, dict(cfg, area_m=((x0 - mg, x1 + mg), (y0 - mg, y1 + mg))))
    known = np.isfinite(Pw[..., 0])
    if (m & known & ~inside).sum() > CLIP_OUT_FRAC * max(m.sum(), 1):
        return True
    ring = _dilate(m, CLIP_RING_PX) & ~m
    with np.errstate(invalid='ignore'):
        tall = h_mm[v0:v1, u0:u1] > max(OBST_MIN_MM, CLIP_H_FRAC * h_c)
    return bool((ring & known & ~inside & tall).any())


def _mark_overlaps(blocks):
    """맞닿은 두 블록 중 위에 걸친 쪽을 top, 덮인 쪽을 under 로 표시한다(입력 dict 를 고침).

    A 와 B 가 영상에서 CONTACT_PX 안으로 맞닿고, 맞닿은 곳에서 B 윗면 − B 의 위 치수(= B 밑면)가 A 윗면 − UNDER_TOL_MM 보다 높으면
    B 가 A 위에 있다(포개짐 · 걸침). 옆에 붙은 더 높은 블록(옆세움 · 세움)은 밑면이 작업면이라 걸리지 않는다."""
    size_mm = None
    for a in blocks:
        a['_grow'] = _dilate(a['mask'], CONTACT_PX)
    for a, b in itertools.permutations(blocks, 2):
        contact = a['_grow'] & b['top']
        if contact.sum() < 10:
            continue
        size_mm = size_mm or [s * 1000.0 for s in a['cfg_size_m']]
        xyz = a['P'][contact]
        zb = xyz[:, 0] * b['coef'][0] + xyz[:, 1] * b['coef'][1] + b['coef'][2]
        b_bottom = (float(np.median(zb)) - a['table_z']) * 1000.0 - size_mm[DIM[b['up']]]
        if b_bottom >= a['h_c'] - UNDER_TOL_MM:
            a['under'] = True
            b['top_of'] = True
            b['on'].append(a)
    for a in blocks:
        a.pop('_grow', None)


def _gaps(blk, P, h_mm, valid, slope, others, cfg):
    """블록의 수평 두 축마다 손가락이 내려갈 틈(mm, 양쪽 중 좁은 쪽). 반환 {축: mm}.

    장애물 = 내 마스크(HALO_PX 넓힘) · 내 블록 자리(치수 + FOOT_MARGIN_MM) 밖에서 작업면 위 높이 > 손가락 끝 높이(윗면 − grasp_depth
    − 여유, 최소 OBST_MIN_MM)인 점 중
      · 다른 블록 마스크(OTHER_DILATE_PX 넓힘) 안의 점 — 찾은 블록은 그대로 장애물, 또는
      · 어느 마스크에도 없으면 작업면 위 NONMASK_MIN_MM 보다 높고 완만한(slope < MAX_SLOPE) 점 · 내 윗면 + STEEP_ABOVE_MM 보다 높은 점.
    까닭(W114 s04_002 실측 단면): 내 블록 가장자리 너머로 작업면까지 비스듬히 이어지는 깊이 번짐(면 밖 0~15 mm, 높이 5~13 mm)과
    마스크보다 긴 내 윗면 끝이 장애물로 세져, 떨어져 있는 블록도 틈이 4~7 mm 로 나왔다. 번짐은 어느 마스크에도 속하지 않는다.
    그 점들로 block_checker.height_map(GAP_CELL_M 격자, 셀마다 최고 z)을 만들고, 축 방향 띠(손가락 폭 + 여유) 안에서
    블록 면(중심 ± 치수/2)부터 가장 가까운 장애물 셀까지 거리를 잰다. 깊이 없는 셀은 비었다고 본다."""
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    thr = max(blk['h_c'] - cfg['grasp_depth_m'] * 1000.0 - FINGER_Z_MARGIN_MM, OBST_MIN_MM)
    win = _bbox(blk['mask'], 160, blk['mask'].shape)      # 160 px ≈ 130 mm — 블록 반길이 37 + 찾는 거리 60 mm 보다 넉넉히
    v0, v1, u0, u1 = win
    Pw = P[v0:v1, u0:u1]
    with np.errstate(invalid='ignore'):
        da = (Pw[..., 0] - blk['center'][0]) * 1000.0, (Pw[..., 1] - blk['center'][1]) * 1000.0
        e_long, e_short = (size_mm[DIM[a]] / 2 + FOOT_MARGIN_MM for a in TOP_AXES[blk['up']])
        foot = ((np.abs(da[0] * blk['u'][0] + da[1] * blk['u'][1]) <= e_long)
                & (np.abs(da[0] * blk['v'][0] + da[1] * blk['v'][1]) <= e_short))
        own = _dilate(blk['mask'][v0:v1, u0:u1], HALO_PX) | foot
        hw = h_mm[v0:v1, u0:u1]
        oth = _dilate(others[v0:v1, u0:u1] & ~blk['mask'][v0:v1, u0:u1], OTHER_DILATE_PX)
        loose = ((slope[v0:v1, u0:u1] < MAX_SLOPE) & (hw > NONMASK_MIN_MM)) | (hw > blk['h_c'] + STEEP_ABOVE_MM)
        obst = valid[v0:v1, u0:u1] & ~own & (hw > thr) & (oth | loose)
    half_win = (max(size_mm) / 2 + GAP_SEARCH_MM + 10.0) / 1000.0
    grid, x0, y0 = height_map(Pw[obst], tuple(blk['center']), half_win, GAP_CELL_M)
    n = grid.shape[0]
    cx = x0 + (np.arange(n) + 0.5) * GAP_CELL_M
    cy = y0 + (np.arange(n) + 0.5) * GAP_CELL_M
    X, Y = np.meshgrid(cx, cy, indexing='ij')
    occ = np.isfinite(grid)
    dx, dy = X[occ] - blk['center'][0], Y[occ] - blk['center'][1]
    strip = (cfg['finger_width_m'] * 1000.0 / 2 + STRIP_MARGIN_MM) / 1000.0
    out = {}
    long_axis, short_axis = TOP_AXES[blk['up']]
    for axis, d, p in ((long_axis, blk['u'], blk['v']), (short_axis, blk['v'], blk['u'])):
        a = dx * d[0] + dy * d[1]
        q = dx * p[0] + dy * p[1]
        half = size_mm[DIM[axis]] / 2000.0
        in_strip = np.abs(q) <= strip
        side = []
        for s in (1.0, -1.0):
            dist = (s * a[in_strip & (s * a > 0)] - half) * 1000.0
            side.append(float(max(dist.min(), 0.0)) if len(dist) and dist.min() < GAP_SEARCH_MM else OPEN_GAP_MM)
        out[axis] = round(min(side), 1)
    return out


TILT_FIT_MIN_PTS = 50       # 기울어진 블록 윗면을 다시 맞출 점이 이보다 적으면 평면 칸을 안 낸다(one_pick tilt_pose 와 같은 50 — 그 블록은 기울여 집기 후보에서 빠진다)
TILT_FIT_RESID_M = 0.003    # 다시 맞출 때 평면에서 이보다 먼 점은 빼고 한 번 더(one_pick fit_plane 3 mm, 두 번)
TILT_FIT_INSET_M = (0.008, 0.004)   # 블록 사각형 안쪽만 — 길이 방향 8 mm · 폭 방향 4 mm 안으로(옆면 · 걸친 블록 점 빼기, one_pick 과 같음)


def _tilt_keys(b, P, table_z, cfg):
    """기울어진 블록(tilted)에만 더하는 칸 — 작업 관리자가 기울여 집기 자세(윗면 법선 · 가운데 높이)를 알게.

    10/13 실기에서 확인한 시험 도구(one_pick tilt_pose)와 같은 방법으로 윗면 평면을 다시 맞춘다: 응답 블록 b 의 중심 · yaw 로 만든 사각형 안쪽
    (길이 L/2 − 8 mm · 폭 W/2 − 4 mm) 점 중 작업면 + 4 mm ~ top_z_m + 5 mm 인 점에 z = a·x + b·y + c 를 맞추고(3 mm 넘게 먼 점 빼고 한 번 더),
    윗면 가운데 높이 = 블록 중심 (x, y) 에서의 평면 높이. 마스크 깎기로 맞춘 평면(_measure)은 쓰지 않는다(b1 one_pick 대조 #5).
    입력: b = 응답 블록 dict(x_m · y_m · yaw_deg · top_z_m) · P = base 점 (H, W, 3) m · table_z = 작업면 높이 m · cfg = finder_cfg(size_m).
    출력: {"center_top_z_m": 윗면 가운데 평면 높이 m(소수 4자리), "normal": 윗면 단위 법선 [nx, ny, nz](nz > 0, 소수 4자리)}
    또는 {}(점이 TILT_FIT_MIN_PTS 보다 적음 — 칸을 안 내면 그 블록은 기울여 집기 후보가 아니다). 바깥 영향 없음."""
    L, W = cfg['size_m'][0], cfg['size_m'][1]
    pts = np.asarray(P, float).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    c, s = math.cos(math.radians(b['yaw_deg'])), math.sin(math.radians(b['yaw_deg']))
    dx, dy = pts[:, 0] - b['x_m'], pts[:, 1] - b['y_m']
    along, across = dx * c + dy * s, -dx * s + dy * c
    sel = pts[(np.abs(along) < L / 2 - TILT_FIT_INSET_M[0]) & (np.abs(across) < W / 2 - TILT_FIT_INSET_M[1])
              & (pts[:, 2] > table_z + 0.004) & (pts[:, 2] < b['top_z_m'] + 0.005)]
    if len(sel) < TILT_FIT_MIN_PTS:
        return {}
    A = np.c_[sel[:, 0], sel[:, 1], np.ones(len(sel))]
    z = sel[:, 2]
    for _ in range(2):
        coef = np.linalg.lstsq(A, z, rcond=None)[0]
        keep = np.abs(A @ coef - z) < TILT_FIT_RESID_M
        if keep.sum() < 3:
            return {}
        A, z = A[keep], z[keep]
    n = np.array([-coef[0], -coef[1], 1.0])          # z = a·x + b·y + c 의 위쪽 법선
    n /= np.linalg.norm(n)
    top = float(coef[0] * b['x_m'] + coef[1] * b['y_m'] + coef[2])
    return {'center_top_z_m': round(top, 4), 'normal': [round(float(v), 4) for v in n]}


def blocks_from_masks(masks, depth_m, intr, T_base2cam, cfg, scores=None, info=None):
    """블록 마스크 목록 → find_blocks 응답의 blocks 목록 (공용 뒷단 — YOLO seg 마스크든 예비 마스크든 같다).

    입력: masks = 블록마다 (H, W) bool 마스크(컬러 영상 기준, 깊이는 컬러에 맞춘 것) 목록 · depth_m = (H, W) m(0 = 없음) ·
          intr = (fx, fy, ppx, ppy) px · T_base2cam = 4x4 (m, base ← 카메라) · cfg = finder_cfg() ·
          scores = 마스크마다 신뢰도 0~1(YOLO conf, 없으면 1) · info = dict 를 주면 디버그 값(작업면 높이 · 블록마다 마스크 번호 ·
          빠진 마스크와 이유)을 채운다 — 응답에는 넣지 않는다.
    출력: list of dict(IRD 칸 그대로, 파이썬 기본형만 · NaN 없음), 윗면 높이(top_z_m) 높은 순.
    바깥 영향 없음. 윗면 점이 모자란 마스크(깊이 구멍 · 너무 작음)는 빼고 info['skipped'] 에 적는다. 예외를 내지 않는다(빈 목록 가능).
    """
    depth_m = np.asarray(depth_m, float)
    P = base_image(depth_m, intr, T_base2cam)
    table_z = table_height(P, cfg)
    slope = slope_image(P)
    h_mm = (P[..., 2] - table_z) * 1000.0
    valid = np.isfinite(h_mm)
    scores = [1.0] * len(masks) if scores is None else list(scores)
    found, skipped = [], []
    for i, m in enumerate(masks):
        m = np.asarray(m, bool)
        if m.shape != depth_m.shape or not m.any():
            skipped.append((i, '마스크 크기가 깊이와 다르거나 비었음'))
            continue
        blk = _measure(m, P, slope, table_z, depth_m, intr, cfg, float(scores[i]))
        if blk is None:
            skipped.append((i, f'윗면 점 {MIN_TOP_PTS}개 미만'))
            continue
        blk.update(index=i, P=P, table_z=table_z, cfg_size_m=cfg['size_m'])
        found.append(blk)
    _mark_overlaps(found)
    others = np.zeros(depth_m.shape, bool)       # 받은 마스크 전부(윗면 점이 모자라 뺀 것도) — 틈 셀 때 '다른 블록' 장애물 후보
    for m in masks:
        if np.shape(m) == depth_m.shape:
            others |= np.asarray(m, bool)
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    out = []
    for blk in found:
        _refine_top(blk, cfg)
        e_long, e_short = (size_mm[DIM[a]] for a in TOP_AXES[blk['up']])
        blk['area_ratio'] = blk['vis_area'] / (e_long * e_short)
        blk['clipped'] = _clipped(blk['mask'], P, h_mm, blk['h_c'], cfg)
        conf = blk['base_conf'] * math.exp(-blk['cost'] / 8.0) * (0.5 if blk['mismatch'] else 1.0)
        if blk['clipped']:
            conf = min(conf, CLIP_CONF)
        blk['confidence'] = float(min(max(conf, 0.0), 1.0))
        gaps = _gaps(blk, P, h_mm, valid, slope, others, cfg)
        # 덮였거나(맞닿은 위 블록 · 보이는 면이 작음) · 블록 하나 모양이 아니거나(MAX_COST) · 윤곽이 잘렸으면(자세 · yaw 를 못 믿음)
        # 집지 않고 장애물로만 쓰게 under
        under = blk['under'] or blk['area_ratio'] < UNDER_AREA_RATIO or blk['cost'] > MAX_COST or blk['clipped']
        top = blk['top_of'] or blk['base_mm'] > 0
        yaw = wrap_deg(math.degrees(math.atan2(blk['u'][1], blk['u'][0])))
        out.append((blk['index'], {
            'x_m': round(float(blk['center'][0]), 4),
            'y_m': round(float(blk['center'][1]), 4),
            'top_z_m': round(blk['z_hi'] if blk['tilted'] else blk['z_c'], 4),   # tilted = 가장 높은 점(E-68 장애물 상자), 나머지 = 윗면 가운데
            'yaw_deg': round(yaw, 1) if round(yaw, 1) < 90.0 else -90.0,
            'up': blk['up'],
            'clear': {k: bool(g >= cfg['min_gap_mm']) for k, g in gaps.items()},
            'gap_mm': gaps,
            'overlap': 'under' if under else ('top' if top else 'none'),
            'tilted': bool(blk['tilted']),
            'confidence': round(blk['confidence'], 2),
        }))
        if blk['tilted']:
            out[-1][1].update(_tilt_keys(out[-1][1], P, table_z, cfg))
    out.sort(key=lambda t: -t[1]['top_z_m'])
    if info is not None:
        info['table_z_m'] = table_z
        info['mask_index'] = [i for i, _ in out]
        info['skipped'] = skipped
        info['detail'] = {b['index']: {k: b[k] for k in ('long_mm', 'short_mm', 'h_c', 'tilt_mm', 'area_ratio', 'cost', 'base_mm',
                                                         'mismatch', 'clipped')}
                          for b in found}
    return [b for _, b in out]


# ---------------- 예비 마스크 (엣지 + 깊이, E-47) ----------------
FG_MM = 7.0               # 작업면 위 이보다 높으면 블록 후보 픽셀 (블록 ≥ 14.8 mm, 깊이 잡음 ±3 mm)
JUMP_SLOPE = 1.2          # 높이 기울기(dz/dxy)가 이보다 크면 층 경계 — 높이가 다른 블록 사이를 끊는다
MIN_AREA_MM2 = 200.0      # 이보다 작은 덩어리는 버린다(세움 윗면 25×15 = 370 mm²)
SINGLE_AREA_RATIO = 1.35  # 덩어리 넓이가 눕힘 윗면(75×25)의 이 배를 넘으면 여러 블록이 붙은 것 — 컬러 엣지로 더 가른다
HALF_HEIGHT = 0.5         # 마스크를 조각 높이의 이 비율보다 높은 픽셀로 줄인다(번진 깊이 경계의 가운데 = 실제 모서리)
CANNY_STEPS = ((40, 120), (15, 45))   # 붙은 덩어리를 끊을 컬러 엣지 문턱 — 먼저 강한 엣지만, 안 끊기면 약한 엣지까지


def masks_from_edges(color, depth_m, intr, T_base2cam, cfg, info=None):
    """예비 마스크(E-47): 컬러 엣지 + 깊이로 블록마다 마스크를 만든다. YOLO seg 모델이 없을 때 · 흔들릴 때 쓴다.

    입력: color (H, W, 3) BGR uint8 · depth_m (H, W) m(컬러에 맞춘 것) · intr · T_base2cam · cfg(finder_cfg). info = 디버그 dict(선택).
    출력: list of (H, W) bool 마스크. 바깥 영향 없음.
    방법: ① 흩뿌릴 영역 안에서 작업면 위 FG_MM 보다 높은 픽셀 ② 높이가 급히 바뀌는 곳(층 경계, JUMP_SLOPE)을 끊음
          ③ 덩어리가 눕힘 블록 하나보다 크면(붙은 블록) 컬러 Canny 엣지로 끊고, 그래도 크면 거리 변환 봉우리로 나눔
          ④ 끊느라 버린 경계 픽셀을 watershed 로 이웃 조각에 돌려준다.
    한계: 같은 높이로 딱 붙은 블록 · 나뭇결 엣지가 강한 블록은 잘못 나뉠 수 있다 — 서로 떨어진 눕힘 블록이 목표(본 방법은 YOLO).
    실패: cv2 가 없으면 ImportError.
    """
    import cv2
    P = base_image(depth_m, intr, T_base2cam)
    table_z = table_height(P, cfg)
    h = (P[..., 2] - table_z) * 1000.0
    with np.errstate(invalid='ignore'):
        fg = np.isfinite(h) & (h > FG_MM) & _in_area(P, cfg)
    k3 = np.ones((3, 3), np.uint8)
    fg = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_OPEN, k3).astype(bool)
    hs = np.nan_to_num(h, nan=0.0).astype(np.float32)
    hs = cv2.medianBlur(hs, 5)
    gx = cv2.Sobel(hs, cv2.CV_32F, 1, 0, ksize=3) / 8.0
    gy = cv2.Sobel(hs, cv2.CV_32F, 0, 1, ksize=3) / 8.0
    d = np.where(np.asarray(depth_m) > 0, np.asarray(depth_m, float), np.nan)
    mm_per_px = np.nan_to_num(d * 1000.0 / intr[0], nan=1.0)
    jump = np.hypot(gx, gy) / mm_per_px > JUMP_SLOPE
    core = fg & ~jump
    gray = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = [cv2.dilate(cv2.Canny(gray, lo, hi), k3) > 0 for lo, hi in CANNY_STEPS]
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    single_max = SINGLE_AREA_RATIO * size_mm[0] * size_mm[1]
    px_area = (np.nan_to_num(d, nan=0.5) / intr[0]) * (np.nan_to_num(d, nan=0.5) / intr[1]) * 1e6   # 픽셀 하나 넓이 mm²

    pieces = []
    n, lab = cv2.connectedComponents(core.astype(np.uint8), connectivity=4)
    for k in range(1, n):
        comp = lab == k
        area = float(px_area[comp].sum())
        if area < MIN_AREA_MM2:
            continue
        if area <= single_max:
            pieces.append(comp)
            continue
        pieces.extend(_split(comp, edges, px_area, single_max, cv2))
    markers = np.zeros(h.shape, np.int32)
    for i, p in enumerate(pieces, start=1):
        markers[p] = i
    markers[~cv2.dilate(fg.astype(np.uint8), k3, iterations=2).astype(bool)] = len(pieces) + 1   # 바깥(작업면)
    if pieces:
        cv2.watershed(np.ascontiguousarray(color), markers)
    masks = []
    for i, p in enumerate(pieces, start=1):
        m = (markers == i) & fg
        # 깊이가 번져 마스크가 블록보다 크다 — 그 조각 높이의 절반보다 높은 픽셀만 남겨 실제 모서리(높이 절반 선)에 맞춘다
        m &= np.nan_to_num(h, nan=0.0) > HALF_HEIGHT * float(np.median(h[p]))
        if px_area[m].sum() >= MIN_AREA_MM2:
            masks.append(m)
    if info is not None:
        info['table_z_m'] = table_z
        info['fg'] = fg
        info['core'] = core
    return masks


def _split(comp, edges, px_area, single_max, cv2):
    """붙은 블록 덩어리 하나를 나눈다: 컬러 엣지(강한 것 → 약한 것 순)로 끊어 보고, 그래도 큰 조각은 거리 변환 봉우리(블록 가운데)를 씨앗으로 나눈다.
    입력 edges = CANNY_STEPS 순서의 엣지 영상 목록. 출력: 조각 마스크 목록(MIN_AREA_MM2 보다 작은 조각은 버림)."""
    if len(edges) > 1:
        cut = comp & ~edges[0]
        n, lab = cv2.connectedComponents(cut.astype(np.uint8), connectivity=4)
        big = [lab == k for k in range(1, n) if px_area[lab == k].sum() >= MIN_AREA_MM2]
        if len(big) <= 1:                            # 강한 엣지로 안 끊김 — 약한 엣지로 다시
            return _split(comp, edges[1:], px_area, single_max, cv2)
        out = []
        for p in big:
            out.extend([p] if px_area[p].sum() <= single_max else _split(p, edges[1:], px_area, single_max, cv2))
        return out
    out = []
    cut = comp & ~edges[0]
    n, lab = cv2.connectedComponents(cut.astype(np.uint8), connectivity=4)
    for k in range(1, n):
        p = lab == k
        area = float(px_area[p].sum())
        if area < MIN_AREA_MM2:
            continue
        if area <= single_max:
            out.append(p)
            continue
        dist = cv2.distanceTransform(p.astype(np.uint8), cv2.DIST_L2, 5)
        seeds = dist > 0.6 * dist.max()
        m, sl = cv2.connectedComponents(seeds.astype(np.uint8))
        if m <= 2:                                   # 봉우리가 하나뿐 — 나누지 못함, 그대로 둔다(뒷단이 under · 비용으로 걸러 낸다)
            out.append(p)
            continue
        mk = np.zeros(p.shape, np.int32)
        mk[seeds] = sl[seeds]
        mk[~p] = m                                   # 덩어리 밖
        img = np.dstack([(dist / max(dist.max(), 1e-6) * 255).astype(np.uint8)] * 3)
        cv2.watershed(img, mk)
        out.extend((mk == j) & p for j in range(1, m) if px_area[(mk == j) & p].sum() >= MIN_AREA_MM2)
    return out


YOLO_DUP_IOS = 0.8         # 두 마스크 중 작은 쪽의 이 비율 이상이 겹치면 같은 블록을 두 번 낸 것 — conf 높은 쪽에 합친다
                           # (W114 검증 장면 실측: 한 블록에 마스크 둘(겹침 0.89~1.00)이 50장 중 9장 · 15쌍.
                           #  다른 블록끼리 맞닿은 마스크 쌍은 겹침 0.05~0.5 라 이 값이면 나뉜다)


def _largest_part(m, use_cv2=True):
    """bool 마스크에서 가장 큰 8-연결 조각만 남긴다. 빈 마스크는 그대로.
    cv2 가 있으면(로봇 PC 노드) connectedComponentsWithStats, 없으면(CI) numpy 로 — 마스크를 감싼 창 안에서 씨앗부터
    '넓히고 마스크와 겹치기'를 더 안 커질 때까지 되풀이해 조각을 하나씩 뗀다(실측 장면당 numpy 약 60 ms · cv2 약 2 ms — 노드는 cv2)."""
    rows, cols = np.flatnonzero(m.any(axis=1)), np.flatnonzero(m.any(axis=0))   # 감싼 창 — _bbox(np.nonzero)보다 빠르다
    if len(rows) == 0:
        return m
    v0, v1, u0, u1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
    out = np.zeros_like(m)
    try:
        import cv2
    except ImportError:
        cv2 = None
    if use_cv2 and cv2 is not None:
        n, lab, st, _ = cv2.connectedComponentsWithStats(m[v0:v1, u0:u1].astype(np.uint8), connectivity=8)
        out[v0:v1, u0:u1] = lab == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        return out
    rest = m[v0:v1, u0:u1].copy()
    best = None
    while rest.any():
        part = np.zeros_like(rest)
        part.flat[np.flatnonzero(rest)[0]] = True
        while True:
            grown = _dilate(part, 1) & rest
            if grown.sum() == part.sum():
                break
            part = grown
        rest &= ~part
        if best is None or part.sum() > best.sum():
            best = part
    out[v0:v1, u0:u1] = best
    return out


def masks_from_yolo(result, hw):
    """YOLO seg 결과 하나(ultralytics Results) → (블록 마스크 목록, 신뢰도 목록). 본 방법(E-38 · E-47)의 ① 단계.

    입력: result = model(...)[0] (masks.data (N, h, w) 0~1 · boxes.conf (N,)) · hw = 컬러 영상 (H, W).
    출력: ([(H, W) bool, …], [float, …]) — 검출 순서 그대로 N 개(합쳐진 중복은 빈 마스크로 둔다 — 뒷단이 '비었음'으로 빼고
          info['skipped'] 에 남는다). 검출이 없으면 ([], []).
    마스크 크기가 (H, W) 와 다르면 가장 가까운 픽셀로 늘린다(numpy 만 — CI 에 cv2 가 없다). 노드는 retina_masks=True 로 불러
    처음부터 원본 크기를 받는다(640×480 은 레터박스 여백이 없어 늘려도 같다). torch 텐서 · numpy 배열 둘 다 받는다. 바깥 영향 없음.
    정리(W086 실측 장면에서 본 것):
      ① 같은 블록을 두 번 낸 마스크(conf 더 높은 마스크와 YOLO_DUP_IOS 이상 겹침)는 그 마스크에 합치고 자기는 비운다 —
         한 블록이 두 블록으로 나가지 않게. 버리지 않고 합치는 까닭: 실측에서 conf 낮은 쪽이 더 넓어 위에 걸친 블록과
         맞닿는 경우가 있었다(s06_035 — 버리면 그 블록의 under 를 놓친다).
      ② 마스크마다 가장 큰 조각 하나만 남긴다 — 경계의 몇 픽셀짜리 부스러기와, 가려 두 조각이 된 아래 블록의 작은 조각
         (E-66: 큰 조각만 그 블록, 두 조각을 한 윤곽으로 잇지 않는다).
      다른 블록끼리 조금 겹친 픽셀은 그대로 둔다 — 깎으면 맞닿음이 줄어 under 를 놓쳤다(s04_004 · s06_011 실측)."""
    def arr(x):
        """torch 텐서 또는 배열 → numpy."""
        x = x.cpu().numpy() if hasattr(x, 'cpu') else x
        return np.asarray(x)
    if getattr(result, 'masks', None) is None or getattr(result, 'boxes', None) is None:
        return [], []
    data = arr(result.masks.data).astype(np.float32)
    conf = arr(result.boxes.conf).astype(float).ravel()
    H, W = hw
    if data.ndim != 3 or len(data) == 0:
        return [], []
    if data.shape[1:] != (H, W):
        rows = np.minimum((np.arange(H) + 0.5) * data.shape[1] / H, data.shape[1] - 1).astype(int)
        cols = np.minimum((np.arange(W) + 0.5) * data.shape[2] / W, data.shape[2] - 1).astype(int)
        data = data[:, rows][:, :, cols]
    raw = [m > 0.5 for m in data]
    scores = [float(c) for c in conf[:len(raw)]] + [1.0] * max(0, len(raw) - len(conf))
    area = [np.count_nonzero(m) for m in raw]    # count_nonzero — bool .sum() 보다 훨씬 빠르다(장면당 마스크 쌍 ~100개)
    masks = [np.zeros_like(m) for m in raw]
    kept = []                                    # 남긴 검출 번호(conf 높은 순)
    for i in sorted(range(len(raw)), key=lambda k: -scores[k]):
        m = raw[i]
        if area[i] == 0:
            continue
        dup = next((k for k in kept if np.count_nonzero(m & raw[k]) >= YOLO_DUP_IOS * min(area[i], area[k])), None)
        if dup is not None:
            masks[dup] |= m
            continue
        kept.append(i)
        masks[i] = m.copy()
    return [_largest_part(m) for m in masks], scores


OVERLAP_BGR = {'none': (0, 200, 0), 'top': (0, 200, 255), 'under': (0, 0, 255)}   # 그림 윤곽 색: 집을 수 있음 초록 · 위 노랑 · 덮임 빨강
TARGET_BGR = (0, 0, 255)     # 받침 목표 자리 상자 색(빨강, 10/14 22시 PL) — 잰 블록 초록보다 굵게 먼저 그려 둘이 겹쳐도 빨간 테가 남게
# 그림 글자의 자세 이름 — cv2.putText 는 한글을 못 그려 영어로(10/14 20시 PL: 눕힘 FLAT · 옆세움 EDGE(잡기 이름 EDGE_SHORT · EDGE_LONG 과 같게) · 세움 STAND)
UP_NAME = {'THICKNESS': 'FLAT', 'WIDTH': 'EDGE', 'LENGTH': 'STAND'}
LABEL_SCALE = 0.4            # 글자 크기(FONT_HERSHEY_SIMPLEX 배율) — 화면을 덜 가리게 작게(10/14 20시 PL), 640×480 에서 블록 하나 폭 정도
BADGE_SCALE = 0.5            # 우선순위 번호 동그라미 글자 크기 — 낱말(LABEL_SCALE)보다 조금 크게, 블록(640×480 에서 폭 ~40 px) 안에 들게
PICK_ALPHA = 0.35            # 고른 블록 마스크 채우기 진하기 — 블록 결이 비쳐 보이게 반투명
AXIS_PX = 25                 # 받침 그림 왼쪽 위 base +x · +y 방향 화살표 길이(픽셀) — 받침 관측 자세는 사진이 돌아가 있어 'dx dy' 방향을 보여 준다


def _deg(a):
    """각도 → 부호 붙은 정수 글자('+12' · '-3' · '+0'). '{:+.0f}' 는 −0.3 을 '-0' 으로 적어서 정수로 반올림해 쓴다."""
    return f'{int(round(float(a))):+d}'


def tilt_deg(b):
    """기울어진 블록의 윗면 기울기 각(°, 정수) — find_blocks 'normal'(윗면 단위 법선, 위쪽) 과 수직(+z) 사이 각. normal 이 없으면 None.
    yaw(돌아간 각)와 다르다 — 그림 글자에 '기울어진 각도'를 적으려고(10/14 22시 PL). 바깥 영향 없음."""
    n = b.get('normal')
    if not n or len(n) != 3:
        return None
    nz = float(n[2]) / max(math.sqrt(sum(float(v) ** 2 for v in n)), 1e-9)
    return int(round(math.degrees(math.acos(min(1.0, max(-1.0, nz))))))


def block_label(b):
    """그림 글자 한 줄: 자세 이름 + 기울었으면 ' TILT n'(기울기 각 °, normal 이 없으면 ' TILT'). 예: 'FLAT' · 'STAND TILT 8'.
    번호는 글자에 넣지 않는다 — 고르는 우선순위는 블록 가운데 동그라미 번호로 따로(draw_found ranks, 10/14 22시 PL: 번호 = 우선순위,
    find_blocks 응답 순서 번호와 헷갈리지 않게). 입력: b = find_blocks 블록 dict(up · tilted · normal). 출력: ASCII 글자. 바깥 영향 없음."""
    name = UP_NAME.get(b['up'], b['up'])
    if not b.get('tilted'):
        return name
    t = tilt_deg(b)
    return f'{name} TILT' + ('' if t is None else f' {t}')


def spare_label(measured, target):
    """받침 그림의 잰 블록 글자: 'SPARE 자세 dyaw ±n'(잰 yaw − 목표 yaw, −90 이상 90 미만). 목표가 없으면 'SPARE 자세 yaw'.
    받침 목표 yaw 가 90°(= −90°) 경계라 잰 yaw 만 쓰면 같은 자세가 +90 · −90 으로 번갈아 보여서 차이로 적는다. 바깥 영향 없음."""
    name = UP_NAME.get(measured['up'], measured['up'])
    if target is None:
        return f'SPARE {name} {_deg(measured["yaw_deg"])}'
    return f'SPARE {name} dyaw {_deg(wrap_deg(float(measured["yaw_deg"]) - float(target["yaw_deg"])))}'


def spare_offset_text(measured, target):
    """받침 그림 왼쪽 위 글자: 'base dx ±n.n dy ±n.n mm'(잰 − 목표 가운데, base_link x · y). 사진 가로 · 세로가 아님을 글자에 적는다."""
    dx, dy = ((float(measured[k]) - float(target[k])) * 1000 for k in ('x_m', 'y_m'))
    return f'base dx {dx:+.1f} dy {dy:+.1f} mm'


def _label_box(cv2, img, txt, org, center=False):
    """_put_label 이 그릴 글자 상자 (x0, y0, x1, y1) 픽셀 — 영상 밖으로 나가지 않게 당긴 자리. 바깥 영향 없음."""
    (w, h), base = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 3)   # 테두리(굵기 3)까지 든 크기
    x0 = org[0] - w / 2 if center else org[0]
    x = max(min(int(x0), img.shape[1] - w - 2), 2)
    y = max(min(int(org[1]), img.shape[0] - base - 2), h + 2)
    return x, y - h, x + w, y + base


def _put_label(cv2, img, txt, org, center=False):
    """img 에 흰 글자(검은 테두리)를 org(글자 왼쪽 아래 — center 면 아래 가운데, 픽셀) 근처에 그린다 — 글자 크기를 재서 영상 밖으로
    나가지 않게 org 를 당긴다. 반환: 그린 글자 상자(_label_box). 바깥 영향: img 를 고친다(호출하는 쪽이 복사본을 넘긴다)."""
    box = _label_box(cv2, img, txt, org, center)
    (_, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 3)
    cv2.putText(img, txt, (box[0], box[1] + h), cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, (0, 0, 0), 3)
    cv2.putText(img, txt, (box[0], box[1] + h), cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, (255, 255, 255), 1)
    return box


def _overlaps(a, b):
    """두 상자 (x0, y0, x1, y1) 가 겹치나. 바깥 영향 없음."""
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def _put_badge(cv2, img, n, center):
    """우선순위 번호 동그라미(흰 바탕 · 검은 숫자 · 검은 테)를 center(픽셀)에 — 영상 밖으로 나가지 않게 당긴다. 바깥 영향: img 를 고친다."""
    txt = str(n)
    (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, BADGE_SCALE, 2)
    r = max(w, h) // 2 + 5
    x = min(max(int(center[0]), r + 1), img.shape[1] - r - 2)
    y = min(max(int(center[1]), r + 1), img.shape[0] - r - 2)
    cv2.circle(img, (x, y), r, (255, 255, 255), -1)
    cv2.circle(img, (x, y), r, (0, 0, 0), 2)
    cv2.putText(img, txt, (x - w // 2, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, BADGE_SCALE, (0, 0, 0), 2)


def draw_found(color, blocks, masks, mask_index, ranks=None, pick=None):
    """find_blocks 결과를 컬러 영상 위에 그린다(IRD `/d2/vision/wrist_image`, E-67 — 화면은 받은 그림을 그대로 보여 준다).

    입력: color (H, W, 3) BGR uint8 · blocks = find 결과(윗면 높이 순) · masks = BlockFinder.last_masks ·
          mask_index = last_info['mask_index'](blocks[k] 가 쓴 마스크 번호) ·
          ranks = 고르는 우선순위대로 늘어놓은 블록 번호(1부터 = blocks[번호 − 1], 작업 관리자 state/2 pick.ranks) — 없으면 번호를 안 그림 ·
          pick = 고른 블록 번호(1부터, pick.index) — 그 블록만 마스크를 칠하고 테두리를 굵게.
    출력: 같은 크기 BGR 새 그림(자르거나 줄이지 않음).
    블록마다 윤곽(overlap 색) + block_label 글자('FLAT' · 'STAND TILT 8')를 윤곽 위쪽 가운데에 작게(블록을 덜 가리게, 10/14 20시 PL) +
    순위가 있으면 가운데에 번호 동그라미(1 = 먼저 집을 블록, 10/14 22시 PL — 순위 없는 블록은 동그라미 없음). find 직후(순위 전)
    그림에는 번호가 없다. 글자 · 동그라미는 영상 테두리 블록이어도 영상 안에 둔다. 바깥 영향 없음. 실패: cv2 가 없으면 ImportError."""
    import cv2
    img = np.ascontiguousarray(color).copy()
    rank_of = {int(n): r for r, n in enumerate(ranks or [], 1)}
    used = []                                    # 이미 그린 글자 상자 — 겹친 블록 글자가 서로 덮지 않게(10/14 시안: 'FLAT' 둘이 겹쳐 'FLATLAT')
    for k, (b, mi) in enumerate(zip(blocks, mask_index)):
        m = np.asarray(masks[mi], np.uint8)
        bgr = OVERLAP_BGR.get(b['overlap'], (255, 255, 255))
        if pick is not None and int(pick) == k + 1 and m.any():   # 고른 블록 — 자기 윤곽 색으로 반투명 채우기
            sel = m.astype(bool)
            img[sel] = (img[sel] * (1 - PICK_ALPHA) + np.array(bgr) * PICK_ALPHA).astype(np.uint8)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cnts, -1, bgr, 4 if pick is not None and int(pick) == k + 1 else 2)
        vs, us = np.nonzero(m)
        txt = block_label(b)
        org = (int(us.mean()), int(vs.min()) - 4) if len(us) else (5 + 60, 20 + 14 * k)
        if len(us) and any(_overlaps(_label_box(cv2, img, txt, org, True), u) for u in used):   # 윗자리가 차 있으면 윤곽 아래로
            (_, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 3)
            org = (org[0], int(vs.max()) + h + 6)
        used.append(_put_label(cv2, img, txt, org, center=True))
        if k + 1 in rank_of and len(us):
            _put_badge(cv2, img, rank_of[k + 1], (us.mean(), vs.mean()))
    return img


def project_to_image(pts_base, intr, T_base2cam):
    """base 점 (N, 3) m → 영상 픽셀 (N, 2) [u, v] (depth_to_base_points 의 역). 카메라 앞(깊이 > 0)이 아닌 점이 있으면 None.
    입력: intr (fx, fy, ppx, ppy) 픽셀 · T_base2cam 4x4 (카메라 → base, m). 바깥 영향 없음."""
    fx, fy, ppx, ppy = intr
    Tc = np.linalg.inv(np.asarray(T_base2cam, float))
    pc = np.asarray(pts_base, float).reshape(-1, 3) @ Tc[:3, :3].T + Tc[:3, 3]
    if (pc[:, 2] <= 1e-6).any():
        return None
    return np.stack([fx * pc[:, 0] / pc[:, 2] + ppx, fy * pc[:, 1] / pc[:, 2] + ppy], axis=1)


def top_face_corners(b, size_m):
    """블록 윗면 네 모서리 base 점 (4, 3) m. 입력: b = 블록 dict(x_m · y_m · top_z_m · yaw_deg · up) ·
    size_m = [길이, 폭, 두께] m. 윗면 첫 변(TOP_AXES[up][0])이 yaw 방향(find_blocks yaw 규칙과 같음). 바깥 영향 없음."""
    a, w = (float(size_m[DIM[ax]]) / 2 for ax in TOP_AXES[b['up']])
    t = math.radians(float(b['yaw_deg']))
    u, v = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
    c = np.array([float(b['x_m']), float(b['y_m'])])
    xy = [c + su * a * u + sv * w * v for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    return np.array([[p[0], p[1], float(b['top_z_m'])] for p in xy])


def draw_spare(color, measured, target, intr, T_base2cam, block_actual_m):
    """받침 다시 재기(find_blocks area spare) 그림: 잰 블록 윗면 사각형(초록) + 목표 자리 상자(빨강) + 글자(wrist_image, 10/14 HMI · PL).

    입력: color (H, W, 3) BGR uint8 · measured = measure_on_spare 블록 dict 또는 None(빈 받침) ·
          target = ideal_spare_block(spare_spots[0]) 블록 dict 또는 None · intr (fx, fy, ppx, ppy) · T_base2cam 4x4 m ·
          block_actual_m = [길이, 폭, 두께] m. 출력: 같은 크기 BGR 새 그림(입력은 그대로).
    그림: 목표 상자(빨강, 4 px)를 먼저, 잰 블록(초록, 2 px)을 위에 — 딱 맞으면 초록 선 양옆에 빨간 테가 보인다.
    글자: 목표 'TARGET'은 상자 아래 · 잰 블록 spare_label('SPARE FLAT dyaw +1')은 사각형 위(테두리를 가리지 않게).
          둘 다 있으면 왼쪽 위에 spare_offset_text('base dx +2.1 dy -0.8 mm') + 그 아래 base +x · +y 방향 화살표
          (받침 관측 자세는 사진이 돌아가 있어 base +x 가 사진 위쪽으로 보인다 — 숫자를 사진 가로 · 세로로 읽지 않게).
    카메라 뒤로 가는 사각형은 그리지 않는다. 바깥 영향 없음. 실패: cv2 가 없으면 ImportError."""
    import cv2
    img = np.ascontiguousarray(color).copy()
    for b, bgr, thick, below in ((target, TARGET_BGR, 4, True), (measured, OVERLAP_BGR['none'], 2, False)):
        if b is None:
            continue
        px = project_to_image(top_face_corners(b, block_actual_m), intr, T_base2cam)
        if px is None:
            continue
        cv2.polylines(img, [np.round(px).astype(np.int32)], True, bgr, thick)
        cu, cv = px.mean(axis=0)
        cv2.drawMarker(img, (int(round(cu)), int(round(cv))), bgr, cv2.MARKER_CROSS, 12 if below else 8, 1)
        if below:
            (_, h), _ = cv2.getTextSize('TARGET', cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 3)
            _put_label(cv2, img, 'TARGET', (cu, px[:, 1].max() + h + 6), center=True)
        else:
            _put_label(cv2, img, spare_label(b, target), (cu, px[:, 1].min() - 6), center=True)
    if measured is not None and target is not None:
        _put_label(cv2, img, spare_offset_text(measured, target), (5, 18))
        dirs = base_axes_px(target, intr, T_base2cam)
        if dirs is not None:
            o = np.array([34.0, 66.0])                                  # 화살표 시작점(왼쪽 위 글자 아래 — 화살표가 위로 가도 글자와 안 겹치게)
            for d, name in zip(dirs, ('+x', '+y')):
                tip = o + AXIS_PX * d
                cv2.arrowedLine(img, (int(o[0]), int(o[1])), (int(round(tip[0])), int(round(tip[1]))), (255, 255, 255), 1, tipLength=0.3)
                _put_label(cv2, img, name, (tip[0] + 6 * d[0], tip[1] + 6 * d[1] + 4), center=True)
    return img


def base_axes_px(target, intr, T_base2cam):
    """target 가운데(x_m · y_m · top_z_m)에서 base +x · +y 로 20 mm 갈 때 사진에서 움직이는 방향 — 단위 벡터 2개 [(du, dv), (du, dv)].
    카메라 뒤 · 같은 점(방향 없음)이면 None. 바깥 영향 없음."""
    c = np.array([float(target['x_m']), float(target['y_m']), float(target['top_z_m'])])
    px = project_to_image(np.array([c, c + [0.02, 0.0, 0.0], c + [0.0, 0.02, 0.0]]), intr, T_base2cam)
    if px is None:
        return None
    d = px[1:] - px[0]
    n = np.linalg.norm(d, axis=1, keepdims=True)
    if (n < 1e-6).any():
        return None
    return d / n


# 작업대 그림(check_progress 답 때) 설계 자리 윤곽 색 — check_progress state 이름 그대로(BlockChecker):
# 있음 초록 · 가려짐 · 모름 주황 · 없음 빨강(10/14 22시 PL, 박진용 안 ②). 표에 없는 값은 주황(모름과 같게)
STATE_BGR = {'present': (0, 200, 0), 'occluded': (0, 165, 255), 'unknown': (0, 165, 255), 'absent': (0, 0, 255)}


def part_short_name(part_id):
    """전체 부품 이름 → 그림 글자용 짧은 이름. 예: '001_CHAIR_BENCH_V000_LEG_001_03' → 'LEG 1-3' · 'LEG_001_03' → 'LEG 1-3'.
    역할[_옵션] = 끝 두 숫자 칸(그룹 · 부품 번호) 앞에서 설계 이름(V 번호 · 숫자 칸)이 나오기 전까지, 번호는 앞 0 을 뺀다.
    꼴이 다르면(끝 두 칸이 숫자가 아님 · 역할이 빔) 이름 뒤 16자 그대로. ASCII 가 아닌 글자는 '?'(cv2.putText 는 한글을 못 그린다). 바깥 영향 없음."""
    toks = str(part_id).split('_')
    name = str(part_id)[-16:]
    if len(toks) >= 3 and toks[-1].isdigit() and toks[-2].isdigit():
        role = []
        for t in reversed(toks[:-2]):
            if t.isdigit() or (t[:1] == 'V' and t[1:].isdigit()):   # 설계 이름 끝(V000) · Template 번호(001)에 닿으면 멈춘다
                break
            role.insert(0, t)
        if role:
            name = f'{"_".join(role)} {int(toks[-2])}-{int(toks[-1])}'
    return name.encode('ascii', 'replace').decode('ascii')


def design_top_corners(center, rot, size_m):
    """설계 자리 윗면 네 모서리 base 점 (4, 3) m — 차례대로 이으면 사각형.
    입력: center = 블록 가운데 (x, y, z) m · rot = 3x3 base←블록(열 = 블록 축의 base 방향 — recipe_parts · BlockChecker.parts 칸) ·
    size_m = [길이, 폭, 두께] m. 윗면 = 위(base +z)를 가장 많이 향한 블록 축의 위쪽 면(BlockChecker · placed_measure.up_of 와 같은 판단).
    yaw 로 바꾸지 않고 rot 를 그대로 써서 세움 · 옆세움 · 기운 설계도 같은 식. 바깥 영향 없음."""
    c = np.asarray(center, float).reshape(3)
    R = np.asarray(rot, float).reshape(3, 3)
    h = np.asarray(size_m, float).reshape(3) / 2.0
    k = int(np.argmax(np.abs(R[2])))
    i, j = (a for a in range(3) if a != k)
    top = c + R[:, k] * h[k] * (1.0 if R[2, k] >= 0 else -1.0)
    ei, ej = R[:, i] * h[i], R[:, j] * h[j]
    return np.array([top + si * ei + sj * ej for si, sj in ((1, 1), (-1, 1), (-1, -1), (1, -1))])


def placed_note(part_id, measured, part_center, rejected=None):
    """A4 다시 재기(find_blocks area assembly) 그림 왼쪽 위 글자(ASCII).
    잼 → 'LEG 1-3 base dx +2.1 dy -0.8 mm'(잰 − 설계 가운데, base_link x · y — spare_offset_text 와 같은 꼴) ·
    못 잼(measured None) → 'LEG 1-3 NOT MEASURED FEW_POINTS'(거절 코드가 있으면 붙임). 입력: part_center = 설계 가운데 (x, y, …) m. 바깥 영향 없음."""
    name = part_short_name(part_id)
    if measured is None:
        return f'{name} NOT MEASURED' + (f' {rejected}' if rejected else '')
    return f'{name} ' + spare_offset_text(measured, {'x_m': part_center[0], 'y_m': part_center[1]})


def draw_workbench(color, parts, intr, T_base2cam, block_actual_m, measured=None, note=''):
    """작업대(조립 자리) 그림: 설계 자리 윗면 사각형을 사진에 겹쳐 그린다(wrist_image — check_progress 답 · find_blocks area assembly 때,
    10/14 22시 PL · 박진용 안 ②).

    입력: color (H, W, 3) BGR uint8 · parts = [{part_id, center (x, y, z) m, rot 3x3, state}] — state = check_progress 판정
          (STATE_BGR: present 초록 · occluded · unknown 주황 · absent 빨강, 2 px — 표에 없는 값은 주황) 또는 None = 다시 재기 목표 자리
          (빨간 4 px 상자 — 받침 목표 TARGET_BGR 와 같게) · intr (fx, fy, ppx, ppy) · T_base2cam 4x4 m · block_actual_m = [길이, 폭, 두께] m ·
          measured = 잰 블록 dict(x_m · y_m · top_z_m · yaw_deg · up — measure_placed) 또는 None → 초록 2 px 윗면 사각형 + 가운데 십자 ·
          note = 왼쪽 위 한 줄 글자(ASCII, 예 placed_note — 비면 안 씀).
    출력: 같은 크기 BGR 새 그림(입력은 그대로).
    차례: 윤곽 전부(목표 상자 → 설계 자리는 윗면이 낮은 것부터 — 사진에서 겹치면 위층이 위에 → 잰 블록) 다음 글자 —
          부품 짧은 이름(part_short_name)을 사각형 가운데에(목표 자리는 상자 아래), 이미 그린 글자와 겹치면 사각형 아래로, 거기도 겹치면 뺀다
          (시작 확인은 부품 전부를 그려 글자가 몰린다). 카메라 뒤로 가는 사각형은 그리지 않는다. 바깥 영향 없음. 실패: cv2 가 없으면 ImportError."""
    import cv2
    img = np.ascontiguousarray(color).copy()
    drawn = []                                   # (부품 dict, 픽셀 사각형) — 글자는 윤곽을 다 그린 뒤에(선이 글자를 긋지 않게)
    tops = []
    for p in parts:
        corners = design_top_corners(p['center'], p['rot'], block_actual_m)
        tops.append((p.get('state') is not None, float(corners[0, 2]), p, corners))
    for _, _, p, corners in sorted(tops, key=lambda t: (t[0], t[1])):   # 목표(state None) 먼저 → 낮은 윗면부터
        px = project_to_image(corners, intr, T_base2cam)
        if px is None:
            continue
        target = p.get('state') is None
        bgr = TARGET_BGR if target else STATE_BGR.get(p['state'], STATE_BGR['unknown'])
        cv2.polylines(img, [np.round(px).astype(np.int32)], True, bgr, 4 if target else 2)
        drawn.append((p, px))
    if measured is not None:
        px = project_to_image(top_face_corners(measured, block_actual_m), intr, T_base2cam)
        if px is not None:
            cv2.polylines(img, [np.round(px).astype(np.int32)], True, OVERLAP_BGR['none'], 2)
            cu, cv = px.mean(axis=0)
            cv2.drawMarker(img, (int(round(cu)), int(round(cv))), OVERLAP_BGR['none'], cv2.MARKER_CROSS, 8, 1)
    used = []
    if note:
        used.append(_put_label(cv2, img, note, (5, 18)))
    for p, px in drawn:
        txt = part_short_name(p['part_id'])
        (_, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, LABEL_SCALE, 3)
        cu, cv = px.mean(axis=0)
        below = (cu, px[:, 1].max() + h + 6)
        tries = [below] if p.get('state') is None else [(cu, cv + h / 2), below]
        for org in tries:
            if not any(_overlaps(_label_box(cv2, img, txt, org, True), u) for u in used):
                used.append(_put_label(cv2, img, txt, org, center=True))
                break
    return img


def _is_int(v):
    """JSON 정수인가(True · False 는 빼고). 바깥 영향 없음."""
    return isinstance(v, int) and not isinstance(v, bool)


def pick_redraw_key(ctx_seq, last_key, pick):
    """작업 관리자 state/2 의 pick 칸으로 손목 그림을 다시 그릴지 정한다(10/14 22시 PL — 고른 블록 · 우선순위 번호를 마지막 공급 find 그림에 덧그림).

    입력: ctx_seq = 노드가 기억한 마지막 공급 find 그림의 image_seq(없으면 None) · last_key = 지난번 다시 그린 열쇠(없으면 None) ·
          pick = state/2 'pick' 칸 {"image_seq": int 또는 null, "index": int(1부터), "ranks": [int, …](1부터, 우선순위 차례), "reason"} 또는 None.
    출력: 다시 그릴 열쇠 (image_seq, index, ranks 튜플) 또는 None(안 그림) — pick 없음 · image_seq null · 기억한 그림과 다른 image_seq
          (그 사이 다시 찾았거나 노드가 다시 켜짐) · 지난번과 같은 열쇠(state 는 다른 칸이 바뀌어도 다시 오므로 같은 그림을 또 내지 않게).
          ranks 가 없으면(null) 빈 목록으로 본다(고른 블록만 칠함).
    실패: pick 이 객체가 아니거나 image_seq · index 가 정수가 아니거나 ranks 가 정수 목록이 아니면 ValueError(노드는 경고만). 바깥 영향 없음."""
    if pick is None:
        return None
    if not isinstance(pick, dict):
        raise ValueError('pick 이 객체가 아님')
    seq = pick.get('image_seq')
    if seq is None:
        return None
    if not _is_int(seq):
        raise ValueError('pick.image_seq 가 정수가 아님')
    if ctx_seq is None or seq != ctx_seq:
        return None
    idx, ranks = pick.get('index'), pick.get('ranks')
    ranks = [] if ranks is None else ranks
    if not _is_int(idx) or not isinstance(ranks, list) or not all(_is_int(r) for r in ranks):
        raise ValueError('pick.index 가 정수가 아니거나 pick.ranks 가 정수 목록이 아님')
    key = (seq, idx, tuple(ranks))
    return None if key == last_key else key


class BlockFinder:
    """find_blocks 계산 묶음: 설정을 들고 있다가 영상 한 장 → blocks 목록. 노드는 이것 하나만 만든다.

    입력: cfg = finder_cfg(robot.yaml dict). 바깥 영향 없음.
    find(color, depth_m, intr, T_base2cam, masks=None, scores=None) → blocks 목록(blocks_from_masks 와 같음).
      masks 가 None 이면 예비 마스크(masks_from_edges)를 만든다. YOLO seg 를 쓰면 그 마스크 목록 · conf 를 넘긴다.
    last_info 에 마지막 계산의 디버그 값(작업면 높이 · 마스크 번호 · 빠진 마스크)을 둔다.
    """

    def __init__(self, cfg):
        """설정 dict 를 받는다(finder_cfg 출력)."""
        self.cfg = cfg
        self.last_info = {}
        self.last_masks = []

    def find(self, color, depth_m, intr, T_base2cam, masks=None, scores=None):
        """영상 한 장에서 블록을 찾는다. 출력: find_blocks 응답 blocks 목록(윗면 높이 순). 실패 때 예외 대신 빈 목록 가능."""
        info = {}
        if masks is None:
            masks = masks_from_edges(color, depth_m, intr, T_base2cam, self.cfg, info=info)
            scores = None
        self.last_masks = list(masks)
        blocks = blocks_from_masks(masks, depth_m, intr, T_base2cam, self.cfg, scores=scores, info=info)
        self.last_info = info
        return blocks
