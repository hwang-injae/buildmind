-- D2 웹 PC DB (PostgreSQL 16) — 설계 · 조립 기록 · AI 호출 기록 (W088). 정본 = PM 폴더 설계_DB_v1 부록 A(10/13 PL 확인판)
-- + 10/14 E-111 형식 4.0(design/4.0 · parts/4.0 — order → sequence) + gen_logs.extra(표에 없는 기록 키 — 스캔 채우기 run_id 등)
-- + 10/14 W197 designs.display_name(사람이 보는 이름 — design_id 는 그대로).
-- db 컨테이너가 데이터 폴더가 비었을 때 한 번만 돌린다(/docker-entrypoint-initdb.d). 키 · 비밀번호는 .env(여기 없음).
-- 저장소 코드(web/backend/design_store.py PgDesignStore)가 이 표만 쓴다 — DB 접근은 그 파일 한 곳.
CREATE TABLE designs (
    design_id        text PRIMARY KEY
                     CHECK (design_id ~ '^[0-9]{3}(_[A-Z]+)+_V[0-9]{3}$'),
    template_id      text GENERATED ALWAYS AS (substring(design_id FROM '^(.*)_V[0-9]{3}$')) STORED,
    family           text GENERATED ALWAYS AS (lower(split_part(design_id, '_', 2))) STORED
                     CHECK (family IN ('chair', 'desk')),
    version          text GENERATED ALWAYS AS (right(design_id, 4)) STORED,
    parent_id        text REFERENCES designs (design_id),
    made_by          text NOT NULL CHECK (made_by IN ('cad', 'web', 'voice', 'scan')),
    prompt           text,
    check_result     jsonb,              -- JSON 키 check(SQL 예약어라 칸 이름만 다름)
    parts            jsonb NOT NULL,     -- parts/4.0
    recipe           json NOT NULL,      -- jsonb 아님: -0.0 · 글자를 그대로 둬야 recipe_sha256 이 맞음(설계_DB 8절 A)
    placements       jsonb NOT NULL,
    ai_template      text,
    ai_reason        text,
    template_changed boolean GENERATED ALWAYS AS (CASE WHEN ai_template IS NULL THEN NULL
                     ELSE left(design_id, length(design_id) - 5) <> ai_template END) STORED,   -- 생성 칸끼리는 서로 못 가리켜 design_id 로
    schema           text NOT NULL DEFAULT 'design/4.0',
    created          timestamptz NOT NULL DEFAULT now(),
    display_name     text CHECK (display_name IS NULL OR length(display_name) BETWEEN 1 AND 40),   -- 사람이 보는 이름(W197 — 고쳐도 됨, 비면 화면은 '벤치 V001')
    CHECK ((made_by = 'cad') = (parent_id IS NULL))
);
CREATE INDEX designs_template_idx ON designs (template_id);
CREATE INDEX designs_family_idx   ON designs (family);
CREATE INDEX designs_parent_idx   ON designs (parent_id);
CREATE INDEX designs_made_by_idx  ON designs (made_by);
CREATE INDEX designs_created_idx  ON designs (created);

CREATE TABLE builds (
    run_id       text PRIMARY KEY CHECK (run_id ~ '^R[0-9]{8}_[0-9]{6}_[0-9a-f]{4}$'),
    design_id    text NOT NULL REFERENCES designs (design_id),
    result       text NOT NULL CHECK (result IN ('DONE', 'STOPPED', 'ERROR')),
    placed       integer NOT NULL,
    total        integer NOT NULL,
    duration_s   real,
    stop_count   integer NOT NULL DEFAULT 0,
    part_errors  jsonb NOT NULL DEFAULT '[]',   -- build/2 키 parts — [{part_id, dx_m, dy_m, dz_m}], 놓은 순서
    schema       text NOT NULL DEFAULT 'build/2',
    saved        timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX builds_design_idx ON builds (design_id, run_id DESC);

CREATE TABLE gen_logs (
    id          bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    kind        text NOT NULL,
    text        text,
    template    text,
    base_id     text,                   -- 사람이 고른 바탕 설계(generate 만)
    model       text NOT NULL,
    prompt_sha  text NOT NULL,
    messages    jsonb,
    calls       jsonb,
    attempts    jsonb,
    result      jsonb,
    fallback    boolean,
    error       jsonb,                  -- {code, message}
    elapsed_s   real,
    started     timestamptz NOT NULL,
    design_id   text,                   -- 실제로 저장된 설계만(설계_DB #14) — 외래 키 아님
    extra       jsonb                   -- 위 칸에 없는 기록 키(스캔 채우기 run_id · nearest_base · feedback 등)
);
CREATE INDEX gen_logs_started_idx ON gen_logs (started);
CREATE INDEX gen_logs_kind_idx    ON gen_logs (kind);

CREATE VIEW templates AS
    SELECT template_id, family, design_id, recipe AS base_recipe, recipe -> 'template' AS template
    FROM designs WHERE version = 'V000';
