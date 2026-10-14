"""PostgreSQL 저장소(web/backend/design_store.py PgDesignStore, W088)가 JSON 폴더 저장소와 같은 모양으로 돌려주는지 지킨다.

시험용 DB 가 있을 때만 돈다 — 환경 변수 D2_TEST_DATABASE_URL(web/db/init/001_schema.sql 을 적용한 빈 DB, 시연 DB 아님 — 표를 비운다).
없으면(CI · psycopg 없음) 건너뛴다. 지키는 것: 레시피 넣었다 꺼내도 recipe_sha256 그대로(json 칸 — 설계_DB 8절 A) · 기본 설계 · 새 설계 ·
목록 모양이 JSON 폴더와 같음 · 생성 칸(template_changed) · 같은 ID 409 · 다음 번호 · save_build 한 번만 · 영구 거절(없는 설계 · run_id 꼴) ·
DB 끊김 = 일시 실패 · AI 기록 번호 · get_design 답이 JSON 으로 바뀜(다리 답).
"""
import copy
import json
import os
import sys
from pathlib import Path

import pytest
import yaml

URL = os.environ.get('D2_TEST_DATABASE_URL')
psycopg = pytest.importorskip('psycopg') if URL else None
pytestmark = pytest.mark.skipif(not URL, reason='D2_TEST_DATABASE_URL 없음 — 시험용 PostgreSQL 이 있을 때만')

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_task'):
    sys.path.insert(0, str(p))

from d2_task.design_checker import DesignChecker  # noqa: E402
from d2_task.parts_to_recipe import PartsToRecipe  # noqa: E402
from d2_task.recipe_document import RecipeDocument  # noqa: E402
from design_store import DesignStore, PgDesignStore  # noqa: E402

RECIPES = ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'recipes'
CFG = yaml.safe_load((ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml').read_text(encoding='utf-8'))
BENCH, TEMPLATE = '001_CHAIR_BENCH_V000', '001_CHAIR_BENCH'
BASES = ('001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000', '003_DESK_STAND_V000', '004_DESK_PEDESTAL_V000')


@pytest.fixture
def pg():
    """표를 비운 시험 DB + 기본 설계 4개."""
    with psycopg.connect(URL) as conn:
        conn.execute('TRUNCATE builds, gen_logs, designs')
    s = PgDesignStore(URL, RECIPES, [75, 25, 15])
    assert sorted(s.register_bases()) == list(BASES)
    return s


@pytest.fixture
def files(tmp_path):
    """같은 기본 설계를 등록한 JSON 폴더 저장소 — 모양 비교용."""
    s = DesignStore(tmp_path, RECIPES, [75, 25, 15])
    s.register_bases()
    return s


def checked(store, made_by='web', ai=None):
    """벤치 V000 블록 그대로를 다음 번호로 진짜 검사(검사 묶음 + 변환기 ①)해 저장한다. 반환: 저장 기록."""
    checker = DesignChecker(CFG)
    checker.parts_to_recipe = PartsToRecipe(checker.grasp_options, [v * 1000 for v in CFG['block_size_m']]).convert
    design_id, _ = store.new_design_id(TEMPLATE, BENCH)
    parts = {'schema': 'parts/4.0', 'design_id': design_id, 'parts': copy.deepcopy(store.get_design(BENCH)['parts']['parts'])}
    _, _, rj = checker.handle_json(json.dumps({**parts, 'base_recipe': store.base_recipe(TEMPLATE)}))
    res = json.loads(rj)
    assert res['ok'] is True, res
    return store.save_design(parts, res['recipe'], res['placements'], BENCH, made_by, prompt='벤치',
                             check={'ok': True, 'min_margin_mm': res.get('min_margin_mm'), 'errors': []},
                             ai_template=ai, ai_reason='까닭' if ai else None)


def no_time(d):
    """시각 칸을 뺀 사본(두 저장소가 저장한 시각은 다르다)."""
    return {k: v for k, v in d.items() if k != 'created'}


def test_bases_round_trip_keeps_recipe_hash_and_shape(pg, files):
    """기본 설계 4개 — 넣었다 꺼내도 레시피 지문이 맞고(로봇 get_design 이 거절하지 않음), JSON 폴더 기록과 칸 · 값이 같다."""
    for design_id in BASES:
        d = pg.get_design(design_id)
        RecipeDocument(d['recipe'], d['placements'])                      # recipe_sha256 · design_id 짝 — 틀리면 ValueError
        assert json.dumps(d['recipe']) == json.dumps(files.get_design(design_id)['recipe'])   # -0.0 · 키 순서까지 글자 그대로
        assert no_time(d) == no_time(files.get_design(design_id))
    assert pg.list_designs() == files.list_designs() and pg.templates() == files.templates()
    assert pg.base_recipe(TEMPLATE) == files.base_recipe(TEMPLATE)


def test_save_design_matches_files_and_db_fills_generated_columns(pg, files):
    """새 설계 — 돌려준 기록 = 꺼낸 기록 = JSON 폴더 기록(시각 빼고). family · version · template_changed 는 DB 생성 칸."""
    rec = checked(pg, made_by='scan', ai='002_CHAIR_BACK')
    assert rec['design_id'] == '001_CHAIR_BENCH_V001' and no_time(pg.get_design(rec['design_id'])) == no_time(rec)
    assert no_time(rec) == no_time(checked(files, made_by='scan', ai='002_CHAIR_BACK'))
    got = pg.get_design(rec['design_id'])
    assert (got['family'], got['version'], got['template_changed']) == ('chair', 'V001', True)
    RecipeDocument(got['recipe'], got['placements'])
    assert pg.new_design_id(TEMPLATE, BENCH) == ('001_CHAIR_BENCH_V002', 'V002')
    assert [r['design_id'] for r in pg.children(BENCH)] == ['001_CHAIR_BENCH_V001']
    assert pg.list_designs(template=TEMPLATE) == files.list_designs(template=TEMPLATE)


def test_same_id_twice_is_value_error(pg):
    """같은 번호를 두 요청이 받아 늦은 쪽이 저장하면 ValueError(REST 409 '다시 생성해 주세요' — 설계_DB 8절 C 가)."""
    rec = checked(pg)
    with pytest.raises(ValueError):
        pg.save_design(rec['parts'], rec['recipe'], rec['placements'], BENCH, 'web', check=rec['check'])


def test_save_build_once_and_permanent_rejects(pg):
    """save_build — 같은 run_id 두 번째는 저장 안 하고 ok · 없는 설계 · run_id 꼴 틀림은 영구 거절(retry false) · 목록에 최근 조립 한 줄."""
    build = {'schema': 'build/2', 'run_id': 'R20261014_101500_a1b2', 'design_id': BENCH, 'result': 'DONE', 'placed': 11,
             'total': 11, 'duration_s': 312.457, 'stop_count': 0,
             'parts': [{'part_id': f'{BENCH}_LEG_001_01', 'dz_m': 0.0012, 'dx_m': None, 'dy_m': None}]}
    assert pg.answer_save_build({**build, 'req_id': 'q1'}) == {'success': True, 'reason': '', 'ok': True}
    assert pg.save_build(build) is False
    assert pg.builds_for(BENCH) == [build]
    assert next(r for r in pg.list_designs() if r['design_id'] == BENCH)['last_build']['max_err_mm'] == 1.2
    for bad in ({**build, 'run_id': 'R20261014_101501_a1b2', 'design_id': '001_CHAIR_BENCH_V099'},
                {**build, 'run_id': 'R1'}, {**build, 'run_id': 'R20261014_101502_a1b2', 'result': 'MAYBE'}):
        res = pg.answer_save_build(bad)
        assert res['ok'] is False and res['retry'] is False, bad


def test_db_down_is_temporary_failure(pg):
    """DB 에 못 붙으면 OSError — save_build 는 retry 없이 ok false(로봇이 다시 보냄), 웹은 시작 때 켜지지 않는다."""
    down = URL.rsplit('@', 1)[0] + '@127.0.0.1:1/d2'
    with pytest.raises(OSError):
        PgDesignStore(down, RECIPES, [75, 25, 15])
    pg.url = down
    res = pg.answer_save_build({'schema': 'build/2', 'run_id': 'R20261014_101500_a1b2', 'design_id': BENCH})
    assert res['ok'] is False and 'retry' not in res


def test_gen_log_number_and_mark(pg):
    """AI 호출 기록 — 번호(글자)를 돌려주고, 고른 후보를 저장하면 그 줄에 design_id 를 적는다(#14). 칸에 없는 키는 extra 에."""
    name = pg.save_gen_log({'kind': 'scan_fill', 'text': '스캔', 'started': 1791900000.5, 'model': 'opus', 'prompt_sha': 'abc',
                            'calls': [{'content': '{"parts": []}'}], 'attempts': [], 'messages': [{'role': 'user', 'content': 'x'}],
                            'run_id': 'R20261014_101500_a1b2', 'nearest_base': BENCH})
    rec = checked(pg)
    pg._mark_gen_log(name, rec['design_id'])
    with psycopg.connect(URL) as conn:
        row = conn.execute('SELECT kind, design_id, extra, calls FROM gen_logs WHERE id = %s', (int(name),)).fetchone()
    assert row == ('scan_fill', rec['design_id'], {'run_id': 'R20261014_101500_a1b2', 'nearest_base': BENCH},
                   [{'content': '{"parts": []}'}])


def test_get_design_answer_is_json(pg):
    """로봇이 다리로 부르는 get_design 답이 그대로 JSON 이 된다(시각이 datetime 이면 다리 json.dumps 가 실패해 시간 초과)."""
    res = pg.answer_get_design({'design_id': BENCH})
    assert res['success'] is True and isinstance(res['created'], float)
    json.dumps(res)
    assert pg.answer_get_design({'design_id': '001_CHAIR_BENCH_V077'})['success'] is False


def test_display_name_on_db(pg):
    """표시 이름(W197) — DB 칸 display_name 만 바뀌고(레시피 · 지문 그대로), 기본 설계를 다시 등록해도 남고, 지우면 칸이 빠진다(JSON 폴더와 같은 모양)."""
    recipe = json.dumps(pg.get_design(BENCH)['recipe'])
    assert pg.set_display_name(BENCH, '젠가 탑 4층') == '젠가 탑 4층'
    pg.register_bases()
    got = pg.get_design(BENCH)
    assert got['display_name'] == '젠가 탑 4층' and json.dumps(got['recipe']) == recipe
    assert next(r for r in pg.list_designs() if r['design_id'] == BENCH)['display_name'] == '젠가 탑 4층'
    with pytest.raises(KeyError):
        pg.set_display_name('001_CHAIR_BENCH_V077', '없음')
    assert pg.set_display_name(BENCH, None) is None and 'display_name' not in pg.get_design(BENCH)
    with psycopg.connect(URL) as conn:
        rows = conn.execute("SELECT design_id, model, extra FROM gen_logs WHERE kind = 'rename' ORDER BY id").fetchall()
    assert rows == [(BENCH, '-', {'old_name': None, 'new_name': '젠가 탑 4층'}), (BENCH, '-', {'old_name': '젠가 탑 4층', 'new_name': None})]
