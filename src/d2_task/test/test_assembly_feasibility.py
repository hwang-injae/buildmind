# -*- coding: utf-8 -*-
"""로봇 조립 가능 검사(W201, assembly_feasibility) 시험 — ROS 없이. R3 · R5 는 실제 DesignChecker 계산을 그대로 쓰는지도 본다."""
import time

import pytest

from d2_task.assembly_feasibility import AssemblyFeasibility
from d2_task.design_checker import DesignChecker
from d2_task.recipe_to_parts import ori_extents
from test_design_checker import BLOCK_MM, CFG, DESIGNS, base_design, design

EXTENT = ori_extents(BLOCK_MM)


def check(parts, snap=False):
    """parts → 판정 결과(robot.yaml 값으로 만든 실제 검사 프로그램을 넘긴다)."""
    return AssemblyFeasibility(DesignChecker(CFG)).check_parts(parts, snap_to_grid=snap)


def rules(res):
    return sorted({f['rule'] for f in res['failures']})


# 기본 설계 V000 4종(시험 사본 레시피 → parts) — R4 없이 R0 ~ R5 로 모두 '가능', 레시피 순서(AI 순서 자리) 그대로 되고 order 도 그 순서다(PL ⑥ R3)
@pytest.mark.parametrize('model_id', DESIGNS)
def test_기본_설계_4종은_레시피_순서_그대로_가능하다(model_id):
    parts = base_design(model_id)['parts']
    res = check(parts)
    assert res['ok'] and res['given_order_ok'] and res['failures'] == []
    assert res['order'] == sorted(p['sequence'] for p in parts)
    assert all(res['grasps'][p['sequence']] for p in parts)


# 같은 층 2 × 2 — 방향 코드 6가지(눕힘 · 옆세움 · 세움 × 좌우 · 앞뒤) 모두 어떤 순서로도 불가: 마지막 블록의 손가락 자리가 끝 · 옆 다 막힘(R5), 1층
@pytest.mark.parametrize('ori', ['x', 'y', 'xe', 'ye', 'zx', 'zy'])
def test_같은_층_2x2는_방향_6가지_모두_불가(ori):
    dx, dy, _ = EXTENT[ori]
    res = check(design(*[(sx * dx / 2, sy * dy / 2, 0.0, ori) for sx in (-1, 1) for sy in (-1, 1)])['parts'])
    assert not res['ok'] and res['order'] is None and rules(res) == ['R5']
    assert {f['layer'] for f in res['failures']} == {1} and all(' → ' in f['detail'] for f in res['failures'])


# 2 × 2 를 품은 3 × 2 · 3 × 3 도 불가(R5)
@pytest.mark.parametrize('cols, rows', [(3, 2), (3, 3)])
def test_2x2를_품으면_불가(cols, rows):
    res = check(design(*[(75.0 * c, 25.0 * r, 0.0, 'x') for c in range(cols) for r in range(rows)])['parts'])
    assert not res['ok'] and rules(res) == ['R5']


# L 자(눕힘 3개): AI 순서는 모서리 블록이 마지막이라 R5 로 막히지만, 모서리를 먼저 놓는 다른 순서는 된다 → ok · given_order_ok False · 되는 순서를 돌려줌(E-121 ⑨)
def test_AI_순서는_안_되지만_다른_순서로_되면_그_순서를_돌려준다():
    parts = design((37.5, 0.0, 0.0, 'x'), (-37.5, 25.0, 0.0, 'x'), (-37.5, 0.0, 0.0, 'x'))['parts']
    res = check(parts)
    assert res['ok'] and not res['given_order_ok'] and res['failures'] == []
    assert sorted(res['order']) == [1, 2, 3] and res['order'] != [1, 2, 3] and res['order'].index(3) < 2
    assert set(res['grasps']) == {1, 2, 3}


# R0: 한 층(밑면 높이 같음)에 방향 코드가 섞이면 순서를 찾지 않고 층 전체 한 줄(part None)
def test_한_층에_자세가_섞이면_R0():
    res = check(design((0.0, 0.0, 0.0, 'x'), (0.0, 100.0, 0.0, 'y'))['parts'])
    assert not res['ok'] and res['failures'] == [dict(res['failures'][0], part=None, layer=1, rule='R0')]


# R2: 아래 블록과 겹친 폭 10 mm(< 15) — 위 블록 R2(2층)
def test_닿는_면이_15mm보다_좁으면_R2():
    res = check(design((0.0, 0.0, 0.0, 'x'), (0.0, 15.0, 15.0, 'x'))['parts'])
    assert not res['ok'] and res['failures'][0]['rule'] == 'R2' and res['failures'][0]['part'] == 2 and res['failures'][0]['layer'] == 2


# R2 경계: 세운 다리 윗면(15 × 25)에 얹는 것처럼 짧은 변이 딱 15 mm 면 된다(003 이 이 경우)
def test_닿는_면_15mm는_된다():
    res = check(design((0.0, 0.0, 0.0, 'zx'), (0.0, 0.0, 75.0, 'y'))['parts'])
    assert 'R2' not in rules(res)


# R3: 위 블록을 35 mm 내밀면 무게중심이 받침 가장자리에서 2.5 mm(< margin_mm 7) — 검사 프로그램 안정성과 같은 판정
def test_내밀어_무게중심이_가장자리에_가까우면_R3():
    parts = design((0.0, 0.0, 0.0, 'x'), (35.0, 0.0, 15.0, 'x'))['parts']
    res = check(parts)
    assert not res['ok'] and res['failures'][0]['rule'] == 'R3' and '2.5 mm' in res['failures'][0]['detail']
    errors = DesignChecker(CFG).check(dict(base_design('001_CHAIR_BENCH_V000'), parts=parts))['errors']
    assert any(e['part'] == 2 and '여유 2.5 mm' in e['detail'] for e in errors)    # 검사 프로그램 안정성(4번)도 같은 여유로 떨어뜨린다


# 받침도 작업면도 없는 블록은 R1(공중)
def test_공중_블록은_R1():
    res = check(design((0.0, 0.0, 0.0, 'x'), (200.0, 0.0, 15.0, 'x'))['parts'])
    assert not res['ok'] and res['failures'][0]['rule'] == 'R1' and res['failures'][0]['part'] == 2


# 스캔 격자 맞춤(판정에만): 벤치 좌표에 0.1 mm 흔들림을 넣으면 그대로는 위층이 공중이지만 snap_to_grid 면 가능. 입력 좌표는 바뀌지 않는다
def test_스캔_좌표는_격자에_맞춰_판정만_한다():
    parts = base_design('001_CHAIR_BENCH_V000')['parts']
    noisy = [dict(p, x=p['x'] + 0.1, y=p['y'] - 0.1, z=p['z'] + (0.1 if p['z'] else 0.0)) for p in parts]
    before = [dict(p) for p in noisy]
    assert not check(noisy)['ok']
    assert check(noisy, snap=True)['ok']
    assert noisy == before


# 54개(check.max_blocks) 한 층 — 2 × 2 를 품은 9 × 6 판(불가)과 띄워 놓은 판(가능) 모두 1초 안(서비스 제한 5초)
@pytest.mark.parametrize('gap, ok', [(0.0, False), (30.0, True)])
def test_54개_한_층도_1초_안에_판정한다(gap, ok):
    parts = design(*[((75.0 + gap) * c, (25.0 + gap) * r, 0.0, 'x') for c in range(9) for r in range(6)])['parts']
    assert len(parts) == CFG['check']['max_blocks']
    start = time.perf_counter()
    res = check(parts)
    assert res['ok'] is ok and time.perf_counter() - start < 1.0


# 입력 parts 를 바꾸지 않는다
def test_입력을_바꾸지_않는다():
    parts = design((37.5, 0.0, 0.0, 'x'), (-37.5, 25.0, 0.0, 'x'), (-37.5, 0.0, 0.0, 'x'))['parts']
    before = [dict(p) for p in parts]
    check(parts)
    assert parts == before
