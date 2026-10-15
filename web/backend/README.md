# web/backend — 웹 PC backend 파일 (FastAPI · MQTT · 설계 AI · 저장소)

> 10/16 최종 정리(main c0d5c71 기준). 이 폴더의 파일 목록만 짧게 적는다 — 흐름 · REST · 저장소 · 실행은 [web/README](../README.md)가 정본(1-B 실행 · 1-C 설계 AI 중계 · 2장 기능 ↔ 통신 · 4장 AI 흐름 · 5장 backend 파일 · 'DB 켜기 · 끄기').
> 옛 10/7 AI 설계 노트(W108 — 템플릿 길 · `templates.py` · `check_local.py` · `tune_system.txt` · `free_system.txt` · 스키마 JSON 2개 · `base_designs/` · `trial_w108.py` · `tests/web/` · `gpt-4o`)는 10/8 E-69(설계도 직접 작성) · 10/13 E-99(설계 AI = Claude)로 그 파일들이 없어져 지웠다(옛 판은 git 기록).

| 파일 · 폴더 | 하는 일 |
|---|---|
| `app.py` | `WebApp` — 앱 만들기 · routes 등록 · `frontend/out` 정적 서빙 · `load_rules()`(`robot.yaml` 설계 규칙 키만 읽기 전용, E-78) · 저장소 열기(`DATABASE_URL`이 있으면 PostgreSQL, 없으면 JSON 폴더) · 기본 설계 4개 다시 등록 |
| `routes/` | `robot.py`(`/api/robot` — 명령 · 정지 · 다시 시작 · 상태 · 그림 `wrist.jpg` · `scan.jpg` · `scan_preview.ply` · `scan_cloud.ply`) · `designs.py`(`/api/designs` — 목록 · 상세 · 규칙 숫자 · 모양 이름 · 표시 이름 · 조립 기록 · 생성 · 후보 고르기 · 스캔 저장 · 스캔 추천) · `ws.py`(`/ws`) · `voice.py`(`/api/voice/listen`) |
| `mqtt_client.py` | `MqttClient` — 브로커와 통신 한 곳(`…/req` · `…/res` + `req_id`, 자동 재전송 없음 · 상태 마지막 값 · 웹 연결 신호 · 다리 연결 감시 · 로봇 PC의 `get_design` · `save_build`에 답). 토픽 규칙은 `src/d2_bridge/d2_bridge/bridge_codec.py`를 같이 쓴다 |
| `design_gen.py` | `DesignGenerator` — 설계 AI 호출 한 곳(Claude 중계 `D2_LLM_URL`, E-99): 생성(후보 3개 → 검사 → 사람이 고른 1개 저장) · 스캔 [그대로 저장]의 AI 칸 채우기(`scan_fill`) |
| `design_store.py` | `DesignStore`(JSON 폴더) · `PgDesignStore`(PostgreSQL, W088) — 저장 · 꺼내기 한 곳 + 형식 이름 상수 · 기본 설계 등록(`d2_task` 변환기 ② import) · 스캔 추천(`match_scan`) |
| `prompts/design_system.txt` | 설계 AI 시스템 프롬프트(규칙 1 ~ 10 · 역할 목록 · 블록마다 쓰는 칸 · 출력). 숫자 자리(`$margin` · `$finger_t` 등)는 `design_gen`이 `robot.yaml` 값으로 채운다. 후보 JSON 형식은 코드(`design_gen`)가 만든다 |
| `voice.py` | `VoiceListener` — 웹 PC **호스트** 음성(컨테이너 밖): 마이크 버튼 → 시동어 → 받아 적기 → MQTT `d2/hmi/intent` · '정지'면 `d2/safety/stop/req`. 점검 `--check` |
| `mock_robot.py` | 가짜 로봇 PC(MQTT만 — 로봇 없이 화면 시험, `--step` · `--timeout-at`) |
| `requirements.txt` | 파이썬 의존성(web 이미지와 호스트 실행이 같이 씀) |
| `data/` | 저장소에 없음(gitignore) — JSON 폴더 저장소 `designs/` · `builds/` · `gen_logs/`(compose는 `/data`로 연결) |
| `models/` | 저장소에 없음 — 음성 시동어 모델 `hello_rokey_8332_32.tflite`를 웹 PC에 둔다(다른 곳이면 `--wakeword` · `D2_WAKEWORD_MODEL`) |

- 시험은 저장소 `tests/`(`test_web_backend` · `test_design_gen` · `test_design_store_pg` · `test_scan_recommend` · `test_claude_bridge` · `test_voice`) — [tests/README](../../tests/README.md).
