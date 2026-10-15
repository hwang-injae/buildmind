# web — HMI 설계 노트 (10/8 · 10/9 황인재, PL 결정)

> 정본은 [IRD](../docs/02_인터페이스_IRD_v3_101315.md) 2 · 4 · 6 · 8 · 10장과 [SDD](../docs/03_설계_SDD_v3_101321.md) 3.1.1 · 6.6 · 6.8 · 6.10장, 결정 E-67 · E-69 · E-70 ~ E-77이다. 이 노트는 **웹 안**(화면 · REST · WebSocket · 저장소 · AI 흐름)을 코드로 옮길 때의 세부만 적는다. ROS · MQTT 이름 · 칸은 IRD가 정본이고(E-57 '정함') 여기서 새로 정하지 않는다 — 바꿀 것은 IRD를 고치는 PL 확인 PR로 올린다. 파일 이름은 SDD 3.1.1을 따른다. [backend/README.md](backend/README.md)는 backend 파일 목록만 짧게 적는다.
> **10/16 최종 정리(main c0d5c71 기준).** 웹 PC = 한석형 노트북(E-92 10/15 덧붙임). IP는 그날 두 PC에서 `ip -4 addr show`로 확인해 아래 `<웹IP>` · `<로봇IP>` 자리에 넣는다.

구조(E-32 · E-41): 브라우저 `frontend`(Next.js 정적 `out/`, three.js) ↔ REST + WebSocket `/ws` ↔ `backend`(FastAPI :8000, paho-mqtt) ↔ MQTT 브로커 ↔ 로봇 PC `d2_bridge`. 브라우저는 브로커 · DB · 설계 AI · ROS를 모른다. 저장소는 `web/.env`에 `COMPOSE_PROFILES=db` · `DATABASE_URL`이 있으면 PostgreSQL(`db` 컨테이너, W088), 없으면 JSON 파일 폴더(아래 'DB 켜기 · 끄기').

## 1. 화면 — 한 페이지 `app/page.tsx` + 칸 (SDD 3.1.1)

화면은 페이지 하나다(`app/` = `page.tsx` · `layout.tsx` · `globals.css` — 다른 페이지 · 동적 경로 없음). 위에서 아래로:

| 칸(컴포넌트) | 무엇 | 쓰는 데이터 |
|---|---|---|
| **위 띠**(`ConnectionDots` · `ProgressTop` · `ThemeToggle`) | 연결 점 3개(웹 서버 = 화면 ↔ backend `/ws` · 브로커 · 로봇 PC 다리) + 저장소 글('저장 DB' / '저장 파일') · 가운데 **진행도**(놓은 블록 / 전체 = %, `DONE`이면 100 %) · 오른쪽 **다크 모드 토글**(해 · 달 — 이 브라우저에만 기억 `localStorage`, 처음엔 운영체제 설정) | `/ws` · `bridge/alive` · `GET /api/designs/rules`의 `store` · `task/progress` |
| **끊김 배너**(`ConnectionBanners`) | 웹 서버 · 브로커 · 로봇 PC 끊김을 크게 한 줄(로봇 PC 끊김 = `bridge/alive` 3초 없음 → [조립] · [스캔] 막음) + '급하면 펜던트 비상정지' | 같음 |
| **로봇 패널**(`RobotPanel`) | 가로 한 줄: 상태(`state` 한글 + `message` + 정지 때 **누가 · 왜** — `safety_state.reason` = `STOP_WEB` · `STOP_TASK` · `TIMEOUT`(작업 관리자 시간 초과) · `STOP_VOICE`(음성 '정지', E-118 ⑥ · #171) · `ROBOT_ALARM:<상태>` · `CTRL_C`. `STOP_KEY`는 정지 노드가 받지만 보내는 프로그램이 없어 안 씀 — 10/14 PL) · 로봇 설계 · 그리퍼(폭 · 잡힘) · [스캔] · [취소] · [🎤 말로 정지(듣기 시작)] · [정지] · [다시 시작]. 그때만 쓰는 [계속] · [초기화] · [그리퍼 열기] · [정리 끝]은 3장 | `task/state` · `safety/state` · `gripper/state` |
| **손목 카메라**(`WristCamera` — 왼쪽 큰 칸, 확대 ×1 ~ ×3 · 전체 화면) | 손목 비전이 그려 보낸 JPEG를 그대로(겹쳐 그리기 없음). 그림 세 가지(#183 · 10/14 22시 PL): 공급 구역(블록 자세 낱말 · 고른 뒤 우선순위 번호 · 고른 블록 칠하기) · 받침 다시 재기(목표 상자 · 잰 블록 · dx · dy · dyaw) · 작업대(놓은 부품 판정 색 — A4 다시 재기 때는 설계 자리 + 잰 블록). 그림 아래 고른 까닭 한 줄(`state/2` `pick.reason` — 공급 구역에서 고른 뒤 다음 고르기 · 완성 · 새 출발 전까지 실림). 지난 조립 그림이거나 60초 넘게 묵은 그림은 흐리게(E-120 10/15 12시대) | `vision/wrist_image` · `state/2` `pick` |
| **설계 3D**(`DesignView` + `DesignTree` · `Preview3D` — 오른쪽) | [설계 목록] → **열 보기**(`DesignTree` — Miller columns: 가구 → 기본 설계 → 파생 → 그 파생 … 을 왼쪽에서 오른쪽 열로, `parent_id` · `version` · `made_by`) → 누르면 3D로 **보기만**(평소 역할별 색, E-40). 보고 있는 설계 아래 **[조립]** 한 번 = `select_design` → 이어서 `start`(이 누름이 출발 확인 SR-09 — 옆에 '작업 영역에 사람 없음 확인', 10/13 PL) · [이름 고치기](표시 이름 W197 — 아래 'DB 켜기 · 끄기'). 그 설계로 조립 중이면 진행 색(아래) | `GET /api/designs` · `/api/designs/{id}` · `task/progress` |
| **AI 생성**(`GeneratePanel` — 설계 3D 아래) | 고른 설계를 바탕(부모)으로 글상자 + 마이크 + [AI 생성](마이크 → 음성 `request_design` 문장이 오면 글 창에 넣고 바로 생성, 사람 확인 단계 없음 — 10/13 PL · AI 모양 추천(classify)은 없앰). 생성 중 → **"디자인을 고르세요"**(후보 3개 3D 나란히 + 검사 결과 · `min_margin_mm`, 불합격 회색) → **"설계도 만드는 중"** → "저장됨 — 위 설계 3D 에서 [조립]" / `GEN_FAILED` 이유 + 기본 설계 권함 / `OUT_OF_SCOPE` 안내 + "AI가 참고한 설계"(E-72) | `/ws` 생성 이벤트(`gen`) · `hmi/intent` |
| **스캔 칸**(`ScanReview` + `CloudView` — `SCAN_MOVE` ~ `SCAN_REVIEW` 동안만) | 스캔 중: 점군 미리보기가 계속 바뀜(live가 3초 안에 왔으면 live, 아니면 자세마다 모은 capture 누적본, E-82). 결과 확인(`SCAN_REVIEW`): 같은 칸에 최종 점군(`cloud_path`가 없거나 빈 글자면 점군 창만 안 띄움, E-35) + 추론 설계 3D(가려져 추정한 블록은 반투명) + 블록 수 · 추정 수 · 가장 비슷한 기본 설계 + [그대로 저장] [다시 스캔] [취소]. 저장이 떨어지면 backend가 준 버튼만(블록 수 초과 = [취소]만 · 그 밖 = [다시 스캔] · [취소]). 손목 실물 RGB · 스캔 사진은 화면에 안 씀(10/13 16시 PL — 화질 · 각도). **촬영 자세 수는 정해 두지 않는다**(`poses_used` · 점군 미리보기 `index` · `pose_id` 그대로 — W166 움직이며 작업공간 전체 스캔이면 3개보다 많다, 10/10 PL). [AI로 고치기]는 보류(10/10 PL — 만들지 않음) | `task/scan_result` · `vision/scan_preview` · `vision/scan_cloud` · `/ws` `scan_save` |
| **추천 설계**(`ScanRecommend` — `SCAN_REVIEW`일 때만, E-97) | 저장된 설계 중 스캔과 가장 비슷한 것(일치율 높은 순 3개 — 비전 `match_rate`와 같은 식, 90 % 아래면 '많이 달라요') · [3D로 보기] · **[조립]**(= `select_design` → `start` — 이미 검사를 통과한 저장 설계라 AI · 새 검사 · 새 저장 없음). 못 받으면 2초 간격으로 3번 다시 받고, 다 실패하면 까닭 + [다시 찾기](10/15 PL) | `GET /api/designs/scan/recommend` |
| **기록**(접힘) | 상태 로그(시각 · 글 — 부품 · 설계 ID는 쉬운 이름으로) | 화면 안 |

- 3D 블록 크기 = `robot.yaml` `block_size_m`(backend가 `GET /api/designs/rules`로 줌 — 10/9 PL 결정 ①). `parts/4.0`(저장 설계) · `blocks/2`(스캔 추론, 위치 · 방향 + `inferred`) 둘 다 같은 칸(`x` · `y` · `z` · `ori`)으로 같은 `Preview3D`가 그린다.
- 진행도 색은 `progress/2.0.parts[].state`(`present` 초록 · `occluded` · `unknown` 주황 · 그 밖 = 아직, 흐린 회색) + 지금 블록(`state/2.part_id`) 노랑 — 안 놓인 블록은 `absent`(첫 관측 전엔 잠깐 `unknown` — 작업 판단 `TaskPlanner`, E-111 A5)라 빨강은 쓰지 않는다(경고로 읽혀서, 10/9). 블록 이름 ↔ 3D 블록은 `placements.steps[].sequence` = `parts[].sequence`로 잇는다. 진행도 색은 `state/2.run_id`와 `progress.run_id`가 같을 때만. **화면은 판정하지 않는다**(dx · dy는 측정값만 — 안 잰 값은 null). % = `present` 수 / 설계 블록 수, `DONE`이면 100 %. 지난 조립 진행표(retained)가 남아 있으면 backend가 비운다 — 들고 있는 `state`의 `run_id`가 비었는데 진행표에는 `run_id`가 있을 때(task를 다시 켠 경우, E-120 10/15 12시대).
- 카메라 표시기는 넣지 않는다 — `camera_status`는 MQTT로 넘기지 않는다(IRD 10.1).

## 1-B. 실행 — 화면 빌드(늘 호스트) · 웹 PC compose · 개발 PC 호스트

**화면은 compose가 만들지 않는다.** web 이미지(`web/Dockerfile` — `python:3.12-slim`)는 파이썬 의존성만 담고(`.dockerignore`가 `frontend`를 뺌), compose가 호스트에서 만든 `web/frontend/out`을 읽기 전용으로 붙인다(`./frontend/out:/repo/web/frontend/out:ro`). 그래서 웹 PC에도 Node 24가 있어야 하고, 화면이 바뀌면 호스트에서 다시 빌드한다.

```bash
# 화면 빌드(웹 PC · 개발 PC 호스트) — Next.js 16.4 + React 19 + Node 24 LTS. Node 는 빌드 때만 쓴다(실행 때 Node 서버 없음).
#   우분투 apt Node 는 18 이라 안 됨 — 사용자 폴더에 nvm 으로:
#   curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.8/install.sh | bash   → 터미널 다시 열고 → nvm install 24
cd web/frontend && npm ci && npm run build            # → web/frontend/out — backend 가 / 에서 내려 준다
```

웹 PC — 저장소 맨 위에서(방화벽 · 처음 셋팅은 아래 '두 PC Wi-Fi 연결'). compose 명령은 `up` · `ps` · `logs` · `restart` 모두 앞의 세 값이 있어야 돈다(`compose.yaml`이 `D2_WEB_IPV4` · `WEB_UID` · `WEB_GID`를 `:?`로 요구 — 빠지면 오류로 끝남):

```bash
mkdir -p web/backend/data                                                                               # 처음 한 번 — web 컨테이너가 쓰는 폴더
D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml up -d --build   # 처음 · requirements 가 바뀌었을 때 --build
D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml ps              # mosquitto · web (+ db) 가 Up
D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml logs -f web     # mosquitto · db 도 같은 꼴
D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml restart web     # main 을 받은 뒤(아래)
```

- main을 받은 뒤 웹 PC 반영: ① 화면이 바뀌었으면 위 화면 빌드 ② `restart web`(코드는 바인드 마운트 · `--reload` 없음 — `up -d`로는 새 코드를 안 읽음) ③ 호스트 프로그램(`claude_bridge.py` · `voice.py`)이 바뀌었으면 그 터미널에서 다시 켬.

개발 PC 호스트에서 직접(compose 없이):

```bash
# backend(FastAPI :8000 — 화면 · REST · /ws). 브로커는 호스트 mosquitto(1883). 화면처럼 localhost 에만 연다(#136 — compose 도 127.0.0.1:8000)
cd web/backend && MQTT_HOST=localhost python3 -m uvicorn app:app --host 127.0.0.1 --port 8000
# 로봇 PC 없이 시험: 가짜 로봇(MQTT) — 조립 · 정지 · 스캔 흉내
python3 web/backend/mock_robot.py --step 1.0
#   --timeout-at 3 = 3번째 부품 집기에서 시간 초과 · 정지 확인 대기 ERROR(stop_pending)를 한 번 흉내 — [다시 시작] 꺼짐 → [정지] → [다시 시작]
# 화면을 고치며 개발(다시 빌드 없이): backend 를 CORS_ORIGINS=http://localhost:3000 으로 띄우고
cd web/frontend && NEXT_PUBLIC_BACKEND=http://localhost:8000 npm run dev
```

브라우저: 웹 PC 에서 `http://127.0.0.1:8000`(화면은 웹 PC localhost 만 — #136, LAN 에 안 열림). 로봇 PC 에서는 `ros2 run d2_bridge bridge --ros-args -p mqtt_host:=<웹IP>`(`robot.yaml` `mqtt.host`는 localhost라 로봇 PC에서는 늘 준다 — 웹 없이 로봇만 시험할 땐 `mock_bridge`).

## 1-C. 설계 생성 AI 중계 `claude_bridge` (10/13 PL 승인 — GPT-4o → Claude)

설계 생성(`design_gen`)은 **웹 PC에 로그인된 Claude Code**로 부른다. 컨테이너 안에는 Claude 로그인이 없으므로 웹 PC **호스트**에서 중계 `web/claude_bridge.py`를 띄우고,
`design_gen`은 OpenAI 라이브러리 그대로 주소(`D2_LLM_URL`)만 중계로 돌린다(키는 보내지 않음). 중계는 OpenAI 형식 요청을 `claude -p --json-schema`로 바꾼다
(도구 · MCP · 파일 접근 없이, 빈 임시 폴더에서). 음성 받아 적기(gpt-4o-mini-transcribe, 안 되면 whisper-1)는 그대로 OpenAI 키를 쓴다.

```bash
# 0) 한 번: 웹 PC 에서 Claude Code 로그인(데스크톱 앱 또는 claude 명령) — 중계는 그 계정의 구독 한도를 쓴다
# 1) 중계 켜기(호스트). compose 의 web 컨테이너가 부르므로 docker0 주소에서만 듣는다(LAN 에는 안 열림)
python3 web/claude_bridge.py --host 172.17.0.1            # 모델은 design_gen 이 보낸다(D2_LLM_MODEL, 기본 opus)
#    컨테이너 없이 backend 를 호스트에서 띄우면: python3 web/claude_bridge.py (127.0.0.1:8790, D2_LLM_URL 비움)
# 2) 확인 — 준비 점검(항목마다 ✅ · ❌(할 일), 다 되면 종료 코드 0). 꼭 --host 172.17.0.1 — 빼면 127.0.0.1:8790 을 봐서 켜져 있어도 '응답 없음'
python3 web/claude_bridge.py --check --host 172.17.0.1   # --no-call 을 더하면 진짜 claude 호출(로그인 · 모델 확인)을 건너뜀
curl -s http://172.17.0.1:8790/health                    # {"ok": true, "claude": "...", "default_model": "opus"}
```

- 모델 고르기: `web/.env`의 `D2_LLM_MODEL`(opus · sonnet — 비우면 opus). 10/13 시험(같은 프롬프트 · 진짜 검사 묶음, 요청마다 1번): 둘 다 벤치 · 책상 후보 3/3 합격 · 다시 만들기 0번,
  생성 시간 opus 46초(벤치) · 136초(책상), sonnet 35초 · 169초. sonnet 은 책상 후보에 블록 35 ~ 54개를 써서(조립이 너무 김) opus 가 기본.
- 중계가 꺼져 있으면 화면에 "설계 AI 중계(claude_bridge)가 꺼져 있어요", 구독 한도면 "Claude 사용 한도에 걸렸어요" — 기본 설계 조립은 그대로 된다.
- `claude`를 못 찾으면 `CLAUDE_BIN=<경로>`(데스크톱 앱이 깐 것은 `~/.config/Claude/claude-code/<판>/<id>/claude` — 중계가 알아서 찾는다).

## 2. 기능 ↔ 통신 대응표

브라우저 → REST → backend → MQTT(`…/req`) → 다리 → ROS 서비스 → 답이 거꾸로. 상태는 MQTT retained → backend → `/ws`로 브라우저에 밀어 준다. REST는 SDD 3.1.1 `routes/` 파일로 나눈다(`robot.py` · `designs.py` · `ws.py` · `voice.py`).

| 화면 기능 | REST(backend) | MQTT(IRD 10.1) | ROS(IRD 4장) |
|---|---|---|---|
| 설계 선택 | `POST /api/robot/command {cmd: select_design, design_id}` | `d2/hmi/command/req` → `/res` | `/d2/hmi/command` |
| 출발 · [계속] | 같음 `{cmd: start, design_id, mode: auto}` | 같음 | 같음(`WAIT_SUPPLY` · `WAIT_HMI`에서 start = 계속) |
| 스캔 · 다시 스캔 | 같음 `{cmd: scan}` | 같음 | 같음 |
| 취소 | 같음 `{cmd: cancel}` | 같음 | 같음(`READY` · `ERROR` · `SCAN_REVIEW`) |
| 초기화(정지 뒤) · 그리퍼 열기 | `STOPPED` · 잠김에서 [정지] 자리가 [초기화] → 한 번 더 확인 → `{cmd: cancel}` → 받으면 '블록을 모두 치워 주세요'(그리퍼 · 조립 자리 · 받침 위 — 10/13 20시 PL 문구, 잠금 유지) → [정리 끝] → `POST /api/robot/resume`. 잠김 + 그리퍼가 쥐고 있으면 [그리퍼 열기] → '블록을 손으로 받쳤나요?' → `{cmd: gripper_open}`(W183 · W184, 10/13 16시 PL) — 여는 중 · 끝(`gripper_opening` · `gripper_done`, 실패 까닭 포함) 문구는 작업 관리자 message 그대로 상태 칸 · 알림에 | 같음 | 작업 관리자가 STOPPED(잠김)에서 cancel 을 받아 기록만 STOPPED 로 닫고 잠금 유지(W183 한석형) → [정리 끝]의 resume 뒤 RECOVER → IDLE. gripper_open 은 STOPPED 일 때만 /d2/gripper/command(0.0865 m · 10 N). cancel 이 거절되면 잠금을 풀지 않음 |
| 정지 | `POST /api/robot/stop` | `d2/safety/stop/req` (`source: web`, **reason 빈 값** → 정지 노드가 `STOP_WEB`) | `/d2/safety/stop` |
| 다시 시작 | `POST /api/robot/resume` | `d2/safety/resume/req` | `/d2/safety/resume` — 잠금 · `STOPPED` · `ERROR`에서 보임(ERROR 에서도 작업 관리자가 RECOVER 로 이어 감). 단 시간 초과 뒤 정지 확인 전 ERROR(`message_id` `stop_pending`, 10/14 PL 안 A)는 작업 관리자가 무시하므로 끄고 '[정지]를 누른 뒤 [다시 시작]' 안내 |
| (그리퍼가 블록을 쥠) | 대기 · 출발 대기 · 완성 · 오류 · 스캔 확인에서 `d2/gripper/state` grasped 면 [조립] · [스캔] · 추천 [조립]을 끄고 '[정지] → [그리퍼 열기] → [다시 시작]' 안내(10/14 HMI 점검 — 작업 관리자 시작 점검이 설계 고르기 · 출발 · 스캔을 거절하고, 오류에서 [다시 시작]은 RECOVER 가 다시 ERROR). [초기화] 알림에서 쥔 채 [정리 끝]을 누르려 하면 경고 | — | — |
| 상태 · 진행도 · 정지 · 그리퍼 · 스캔 결과 | `GET /api/robot/state`(지금 값) + `/ws` | `d2/task/state` · `progress` · `scan_result` · `d2/safety/state` · `d2/gripper/state`(retained) | 토픽 |
| 로봇 PC 끊김 | `/ws` `{type: bridge_alive, alive}` | `d2/bridge/alive`(LWT `alive: false`) | — |
| 버튼 기다림 시간 | `/ws` 붙을 때 `{type: timing, req_timeout_s}` — 화면은 이 값 + 3초까지 기다림(숫자를 화면 코드에 두지 않음, 10/9). 아직 못 받았으면 보내지 않고 안내 | — | — (`robot.yaml` `mqtt.req_timeout_s`) |
| 웹 연결 신호 | backend가 1초마다 냄 | `d2/web/alive` | 다리 → `/d2/hmi/alive` |
| 마이크 버튼 | `POST /api/voice/listen` → `{success, reason}` | `d2/hmi/voice/listen` `{schema: voice_listen/1, stamp}`(QoS 1, retained ✗ — 웹 PC 안, 다리 안 거침) → voice.py 듣기 대기 → 시동어 'hello rokey' → 녹음 · 받아 적기(E-105 — 시동어 뒤 '정지'는 정지 요청, '멈춰' · '스톱' · '그만'도 받음 — 10/14 PL 대표 낱말 '정지') | — |
| 음성 문장 | `/ws` `{type: intent, …}` — 화면은 마이크를 누른 뒤 온 `request_design`만 받음. 생성은 `POST /api/designs/generate`에 `made_by: voice` · `confidence`(받아 적기 확신도, 없으면 null)를 실어 AI 가 오인식을 바로잡아 읽고, 그래도 모르면 `VOICE_UNCLEAR`('이렇게 들었어요: … — 다시 말씀해 주세요', 10/14 PL #171) | `d2/hmi/intent`(voice.py → 브로커, backend도 구독) | 다리 → `/d2/hmi/intent` |
| 설계 규칙 숫자 | `GET /api/designs/rules` | — | — (backend가 `robot.yaml`을 읽음, 10/9 ①) |
| 설계 생성 | `POST /api/designs/generate {text, design_id}`(design_id = 사람이 고른 설계 = 부모, 없으면 404 — 10/13 PL, 옛 `/template` 없앰) → `{job_id}`(한 번에 하나 — 409), 진행은 `/ws` `{type: gen, job_id, stage: reading · candidates · checked · retry · done · failed · saved, …}` · `GET /api/designs/generate/{job_id}` | 검사만 `d2/task/check_design/req` × 3 | `/d2/task/check_design` |
| 후보 고르기 → 저장 | `POST /api/designs/generate/{job_id}/pick {index, made_by}` → `{design_id, version, parent_id, index}`(떨어진 후보 · 이미 저장 409) | — (검사 결과를 이미 받아 둠) | — |
| 설계 목록(트리) · 1개 | `GET /api/designs`(요약 — 트리는 화면이 `parent_id`로 만듦, `/tree`는 두지 않음) · `GET /api/designs/{id}`(`design/4.0`, 없으면 404) | — | — |
| 조립 기록 | `GET /api/designs/{id}/builds`(build/2 전부, 최근 것 먼저 · 설계가 없으면 404) — 목록 요약에도 `last_build` 한 줄 | — | — |
| 모양 이름 · 표시 이름 | `GET /api/designs/templates` → `[{id, name, label}]`(V000 레시피 `template.name` — 화면이 설계 ID 대신 '벤치 V001'로) · `PUT /api/designs/{id}/name {name}`(표시 이름 W197 — 아래 'DB 켜기 · 끄기') | — | — |
| 스캔 [그대로 저장] | `POST /api/designs/scan/save {run_id}` → `{job_id}`(마지막 `scan_result`와 `run_id`가 다르거나 이미 저장 중이면 409) → backend가 4-C(AI 칸 채우기 `scan_fill`) → check_design(요청에 `made_by: "scan"` — 21번 Template 모양만 건너뜀, E-106 · 생성 요청엔 안 붙음, AI 칸 · Template 탓이면 AI가 다시 채움 최대 2번) → 저장(`made_by scan`). 진행은 `/ws` `{type: scan_save, stage: filled · checked · retry · saved · failed}` · `GET /api/designs/scan/save/{job_id}`(실패면 `code` · `message` · `actions`) → 저장되면 화면이 `select_design` | `check_design/req` · `hmi/command/req` | 같음 |
| 스캔 추천 설계(E-97) | `GET /api/designs/scan/recommend?run_id=` → `{run_id, candidates[{design_id, family, made_by, matched, scan_count, part_count, rate}]}`(저장 설계 전부와 일치율 — 비전 `match_rate`와 같은 식, 높은 순 3개 · 읽기만, `run_id`가 다르면 409). [조립] = `select_design` → `start` | 조립 때만 `hmi/command/req` | 같음 |
| 스캔 [AI로 고치기](보류 — 10/10 PL, 만들지 않음) | `POST /api/designs/generate {text, parent_id: <스캔 설계>}` — 4-B | 검사만 | 같음 |
| 사진 · 점군 | `/ws` `{type: scan_image · scan_cloud, seq, stamp, bytes, run_id}` → `GET /api/robot/scan.jpg` · `/api/robot/scan_cloud.ply`(마지막 스캔 것만 메모리에, 없으면 404). `run_id` = 직전 `scan_result`의 것 — 화면은 `scan_result.run_id`와 같을 때만 점군을 보여 줌(사진은 화면에 안 씀 — 10/13 16시 PL). 새 스캔이 오면 앞 것은 지움(점군 없는 스캔에 옛 점군이 안 남게) | `d2/vision/scan_image`(JPEG ≤ 500 KB) · `scan_cloud`(PLY ≤ 2 MB), QoS 1 | 다리가 `scan_result`의 `image_path`(PNG → JPEG로 줄임) · `cloud_path` 파일을 읽어 `scan_result` 바로 뒤에 보냄 |
| 스캔 중 점군 미리보기 | `/ws` `{type: scan_preview, seq, stamp, bytes, run_id, index, kind, pose_id, points}` → `GET /api/robot/scan_preview.ply?kind=capture\|live`(kind 별 마지막 하나, 없으면 404). 화면 스캔 칸이 스캔 중 실시간으로 바꿔 그림(live 가 3초 안이면 live, 아니면 capture 누적본) — 결과 확인이 되면 같은 칸에 최종 점군 · 추론 설계(10/13). 손목 실물 RGB · 스캔 사진은 화면에 안 씀(10/13 16시 PL — 화질 · 각도) | `d2/vision/scan_preview`(JSON, QoS 1) + 바로 뒤 `scan_preview_cloud`(PLY — capture ≤ 2 MB QoS 1 · live ≤ 500 KB QoS 0), retained ✗. backend 는 바로 앞 JSON 과 짝짓고 옛 index · 새 run_id 를 거름 | 다리가 `/d2/vision/scan_preview`의 `cloud_path` 파일을 읽어 JSON 바로 뒤에 보냄(E-82, 한 쌍은 잠금 안에서) |
| 손목 검출 화면 | `/ws` `{type: wrist_image, seq, stamp}` → `GET /api/robot/wrist.jpg?seq=`(마지막 한 장, 없으면 404) | `d2/vision/wrist_image`(JPEG 바이트, QoS 0) | `/d2/vision/wrist_image` |
| (로봇이 부름) 설계 꺼내기 · 결과 저장 | backend가 `DesignStore`로 답함 | `d2/hmi/get_design/req` → `/res` · `d2/hmi/save_build/req` → `/res` | `/d2/hmi/get_design` · `save_build` |

- 요청 시간 제한 5초(IRD `mqtt.req_timeout_s`). **자동 재전송은 어디에도 없다** — 시간 초과 · `BUSY` · 웹 연결 신호 없음 거절(`success: false` + reason 빈 값, #83) 모두 사람이 다시 누른다. 실제 상태는 `state/2` retained로 보여 준다. 버튼은 응답이나 시간 초과 뒤 바로 풀린다.
- `get_design`은 작업 관리자 `timeout.service_s` 3초 안에 답해야 한다 → backend는 저장소(JSON 폴더 또는 DB)에서 바로 읽는다(AI · 검사 호출 없음). `save_build`는 같은 `run_id`가 또 오면 한 번만 저장하고 `ok: true`.

## 3. 상태별 버튼 (SDD 5.1 상태표)

| `state/2.state` | 켜지는 버튼 | 화면 글 |
|---|---|---|
| `IDLE` | 설계 선택 · 스캔 · 생성 | 설계를 고르세요 |
| `READY` | [조립](설계 3D) · 취소 | [조립]을 누르세요 — "작업 영역에 사람이 없는지 확인하세요" |
| `CHECK` · `SELECT` · `PICK_PLACE` · `VERIFY` | (정지) | 조립 중 n/N (%) |
| `WAIT_SUPPLY` | 계속(= start) | `supply_empty` · `tilted_block` · `no_match_block`("맞는 블록이 없어요. 블록을 정리하거나 더 넣고 [계속]", E-73 — 받침 경유 · 공급 사람 확인도 이것) · `placed_check`(A4 모서리 판정 `w144_corner_judge` 켬의 사람 수정 대기 — 모서리 안쪽 칸 높이 차 · TILTED · tilted. 화면 "놓은 블록 확인 — 손으로 고친 뒤 계속" · [고쳤어요 · 계속] → 그 블록을 다시 잼, 10/15 PL · 한석형 #190) |
| `WAIT_HMI` | 계속(= start) | 웹 연결이 끊겼었어요. [계속]을 누르세요(`hmi_lost`) |
| `SCAN_MOVE` · `SCAN_CAPTURE` · `SCAN_INFER` | (정지) | 스캔 중(`scan_running`) |
| `SCAN_REVIEW` | 그대로 저장 · 다시 스캔 · 취소 · 추천 설계 [조립] — E-97 · [AI로 고치기]는 보류 | 스캔 칸 비교(`scan_review`). 스캔한 설계가 검사에 떨어지면(블록을 잘못 읽어 W201 '로봇 조립 불가'인 때 포함) 저장은 막힌다 → [다시 스캔] 또는 추천 설계 [조립] — 003은 다시 스캔해도 같아 추천 설계만(E-121 ⑭ · 10/15 16시대) |
| `STOPPED` | 다시 시작 · 초기화([정지] 자리) | 멈췄어요 — `reason` + "로봇 작업 영역에서 손을 빼고 누르세요" |
| `RECOVER` | (정지) | 다시 시작 중(저속) |
| `ERROR` | 다시 시작 · 취소 | 오류 — `message`(그리퍼 응답 없음 · 피드백 없음도 여기, E-74 · E-77) |
| `DONE` | 설계 선택 · 스캔 · 생성 | 완성(`done`) · 100 % |
| 로봇 PC 끊김(어느 상태나) | 정지만(닿지 않을 수 있어 '급하면 펜던트 비상정지' 안내) | 배너 |

[정지]는 어느 상태에서나 켜져 있다(크고 빨강).

### 3-1. 화면 안내가 나오면 — 확인 방법 (10/14 17시 PL: 화면에는 '문제 — 할 일' 한 줄만, 명령 · 프로그램 이름은 여기)

웹 PC 명령은 저장소 맨 위에서 친다(`<웹IP>` = 웹 PC Wi-Fi IPv4 — 1-B). 화면 글 속 부품 · 설계 ID(`001_CHAIR_BENCH_V000_LEG_001_03`)는 화면이 '다리 1-3' · '벤치 (기본)'으로 바꿔 보인다(`lib/names` `readable` — 작업 관리자 글은 그대로 옴).

| 화면 안내 | 확인할 것 | 확인 방법 |
|---|---|---|
| 웹 서버 연결 끊김 | 웹 PC `web` 컨테이너 | 웹 PC `D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml ps` · `D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml logs web` |
| 브로커 연결 끊김 | 웹 PC `mosquitto` | 웹 PC `D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml ps` · `D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml logs mosquitto` |
| 로봇 PC 연결 끊김 | 로봇 PC 다리(`d2_bridge`) · 네트워크 | 로봇 PC 터미널 다리 로그, 로봇 PC 에서 웹 PC IP 로 `ping` |
| 로봇 PC 답이 늦어요 · 로봇 PC에서 실패 | 작업 관리자 · 다리가 살아 있는지 | 로봇 PC 작업 관리자 터미널 로그 |
| 설계 AI 중계가 꺼져 있어요 · 호출 실패 | 웹 PC 호스트 `claude_bridge` | 웹 PC `python3 web/claude_bridge.py --check --host 172.17.0.1`(`--host`를 빼면 127.0.0.1을 봐서 켜져 있어도 '응답 없음') · 중계 로그 |
| 설계 검사를 못 했어요 | 로봇 PC 작업 관리자(검사 묶음) · 다리 | 로봇 PC 터미널 로그 |
| 검사 요청 오류(웹) | 기본 설계(V000) 레시피가 저장소에 맞게 있는지 | backend 로그 · web 컨테이너 다시 켜기(1-B `restart web` — 기본 설계 다시 등록) |
| 저장소에 못 썼어요 | DB · 데이터 폴더 | backend 로그 |
| 음성을 못 받았어요 | 시동어 · 마이크 · `voice.py` | 웹 PC 터미널 `voice.py` 로그 · 마이크 연결 |
| 잠금을 못 풀었어요 | 펜던트 비상정지 · 로봇 알람 · 정지 노드 | 상태 칸 정지 까닭 · 펜던트 화면 |
| 그리퍼를 못 열었어요 | 멈춤(잠금)인지 · 그리퍼 연결 · 전원 | 상태 칸 '멈춤' · 아래 '그리퍼' 줄(신호 없음) |

## 4. AI 흐름 (E-67 · E-69 · E-72, 10/9 PL)

### 4-A. 생성 — 후보 3개 → 사람이 1개

**범위(10/10 PL):** 가구는 의자 · 책상 둘만(기본 설계 4개 = Template 4개 — E-84). 그 밖의 가구 · 예외 상황은 시나리오에서 뺀다 — 프롬프트도 의자 · 책상만, 다른 가구 요청은 `OUT_OF_SCOPE`.

```text
①  설계 목록에서 바탕 설계를 고름 → 글 창 [AI 생성]  또는  마이크 → 음성 request_design 문장 → 글 창에 넣고 바로 생성(10/13 PL)
②  바탕(10/13 PL): 고른 설계 = 새 설계의 parent_id, Template = 그 앞부분. 요청 글에 고른 설계의 블록 목록을 넣고,
    같은 Template 의 다른 설계는 store.list_designs 요약 + 도구 get_design(첫 차례 tool_choice required + 선택지 SKIP '안 읽음', 그다음 auto — 중계가 auto 면 도구를 버리고 바로 답해 읽기는 사실상 한 번, 10/14)으로 참고만(E-72)
    → 화면 "바탕 … · 참고한 설계 …". 도구 단계가 실패하면 examples_for(template)(바탕 설계는 뺌). 다른 설계가 없으면 도구 없이 바로
③  Claude 1번 호출 → 후보 3개(parts/4.0 — 부품마다 sequence · x · y · z · ori + role · group · stage · grasp, 구조화 출력 candidates[3])
    프롬프트: 역할 · 옵션 목록(src/d2_task/d2_task/roles.json) · 잡기 규칙(SDD 6.6) · 설계 규칙 숫자(robot.yaml — 10/9 ①) · RAG 예시 · 요청 글에 [Template 규칙](그 V000 레시피 composition 의 count · revision 을 사람 말로 — 21번이 보는 것, 10/14 PL)
    + W201 조립 규칙 8 ~ 10(한 층 같은 방향 · 아래와 닿는 면 15 mm 이상 · 같은 층 판처럼 붙이기 금지) · 손가락 자리 숫자 $finger_t · $finger_w · $grasp_depth
      (robot.yaml — 검사 묶음과 같은 값, E-121 ⑲)
    칸이 빠지거나 틀리면 check_design이 CHECK_FAILED — 코드가 대신 채우지 않음(E-69 ②). OUT_OF_SCOPE → 안내, 끝
④  화면에 3개 3D + "디자인을 고르세요" — 동시에 check_design × 3(차례로, 각 3초 목표) → 오는 대로 합격 · min_margin / 불합격 회색 + detail
⑤  3개 다 불합격 → errors[].detail 피드백으로 ③부터(최대 2번) → 그래도 실패 GEN_FAILED(이유 + 기본 설계 권함)
    생성 전체 상한 5분(GEN_LIMIT_S — 10/14 PL): 남은 시간 < 60초면 더 만들지 않음 · 호출 제한 = 남은 시간. 하나도 못 통과하면 떨어진 까닭(만들 수 없는 모양 등)을 보여 줌 — 더 만들기 호출이 시간 초과여도 그것으로 덮지 않음
⑥  사람이 1개 고름 → "설계도 만드는 중" → 그 후보의 check_result/3.0(recipe · placements)로 save_design
    (아직 검사 중이면 기다림) → 고른 1개만 designs(안 고른 후보는 메모리에서 버림 — E-84 ⑥, rejected 기록 없음) → "저장됨 — 위 설계 3D 에서 [조립]"
⑦  설계 3D 의 [조립] → hmi/command select_design → 작업 관리자가 get_design → READY → 이어서 hmi/command start(한 번 누름 — 10/13 PL)
```

- 시간(생성 전체 상한 5분 — 10/14): LLM(Claude, 10/13 — 고르기 · 읽기 각 3초쯤 + 후보 작성 1 ~ 2분 반, 호출당 제한 260초 — 중계는 240초에 먼저 끊음) + 검사 3번(≈ 10초). 후보는 backend 메모리(`job_id`)에만 — 새로고침하면 사라져도 된다(저장 전).
- `design_id` = `<Template ID>_V<3자리>`(10/10 E-84 · IRD 2장): Template = 기본 설계 4개 이름(`001_CHAIR_BENCH` …), 기본 = `_V000`, AI · 음성 · 스캔 = 같은 Template의 다음 번호(그 Template의 가장 큰 V + 1 — 부모와 상관없이 만든 차례). 부모 · 만든 방법은 `parent_id` · `made_by`에만, `version` = V 번호(`V001`). 저장 설계는 바뀌지 않는다(E-55 — 고치면 다음 번호). `gen_path` 칸 없음(E-71).
- **ID는 검사 전에 정한다**(10/10, W111): 변환기 ①이 그 ID로 레시피 `design_id` · `part_id`를 만들기 때문. ③ 전에 `store.new_design_id(template, parent_id)`(AI = 고른 설계의 Template, 스캔 = 부모의 Template) → 후보 3개를 같은 ID로 `check_design` → 고른 1개만 `save_design`. 저장 때 다른 요청이 먼저 같은 ID를 쓰면 거절 → ID부터 다시 · 검사도 다시.
- **연짝 잡기 = FLAT_LONG**(E-104, 10/13 PL — 설계 자체를 고침): `check_design`에 보내기 직전 `design_gen._check`가 연짝(같은 층에 긴 면을 맞댄 눕힌 블록 — `d2_task.row_grasp.row_orders`)을 찾아, 놓는 순간 손가락이 들어가는 블록(검사 묶음 `grasp_options`)만 `grasp`를 FLAT_LONG으로 바꾼다(생성 · 스캔 [저장] 둘 다). 검사 · 변환기 ①이 바뀐 잡기로 보고 placements `grasp_axis`도 LENGTH로 나온다. 안 되는 블록은 그대로(생성 기록 `row_grasp.kept`), AI가 처음 쓴 답은 생성 기록 원문에 그대로.
- **참고 설계는 그 Template 안에서만**(E-84 ③): AI(Claude)에게 주는 목록 요약 · 도구 `get_design` 허용 목록 = `store.list_designs(template=…)`, 자동 전환 = `store.examples_for(template)`(그 V000 + 최근 파생 3). 다른 Template 설계를 읽고 만든 후보는 저장에서 거절되기 때문.
- `save_design`이 거절하는 것(저장하지 않음, 마지막 방어선): 검사 불합격(SR-09) · `made_by`가 web · voice · scan 아님 · ID가 E-84 꼴이 아님 · V000(기본 설계 몫) · 모르는 Template · 레시피 짝(`design_id` · `recipe_sha256` — 로봇 작업 판단과 같은 `RecipeDocument` 확인) · 부모 없음 · 부모가 다른 Template. 저장 칸 = `design/4.0` + `prompt` · `check`(ok · min_margin_mm · errors) · `created`(FR-D02).
- 검사 요청 · 응답은 IRD 그대로(`parts/4.0`만 → `check_result/3.0`). **형식 버전 규칙(IRD 6장):** 앞자리 같으면 받고 모르는 칸 무시, 다르면 거절(이유에 받은 schema). 형식 이름 상수는 `design_store.py` 한 곳. 옛 `cad_structure/1.0` · `cad_recipe/1.0` · `blocks/2` 설계는 거절(변환 안 함, E-69 ⑧).

### 4-B. 스캔 → [AI로 고치기] (W125 챌린지, 10/9 PL ③ — **10/10 PL: 10/14 이후로 보류, 지금 만들지 않음**)

스캔 비교 화면의 **[AI로 고치기]를 누를 때만** 돈다(스캔마다 자동으로 돌리지 않음 — 핵심 W116 복제가 AI 시간 · 비용에 묶이지 않게). 스캔 원본을 먼저 4-C로 저장한 뒤 그 설계를 부모로 4-A를 그대로 돈다(요청 문장은 비어도 됨 — "검사에 맞게 고쳐 줘" · "한 층 높여 줘"). 화면 = 스캔 원본 3D 1개 + 후보 3개. 고른 1개는 `made_by web` · `parent_id` = 스캔 설계. 스캔 원본이 검사에 떨어졌을 때도 같은 길(`detail` 피드백 → 후보 3개).

### 4-C. 스캔 → [그대로 저장] (IRD 8.4 ⑦, 10/9 PL ②)

```text
scan_result/2.0 의 blocks/2(위치 · 방향 + inferred, 임시 이름 scan_<family>_NN · family = 가장 가까운 기본 설계 것(모르면 unknown) — 저장 때 버림) + nearest_base(추론기가 고른 기본 설계, 선택 칸 — E-78 ~ E-80)
→ design_gen이 Claude 1번 호출: 4-A ②와 같은 RAG(DB 목록 요약 + 도구 get_design) → 가장 비슷한 설계를 골라 읽음
   → 그 설계를 참고해 role · group · sequence · stage · grasp만 채움(inferred는 '추측' 힌트). 블록 위치 · 방향은 바꾸지 않는다(복제가 목적)
   → 고른 설계 = parent_id, family = AI(Claude) 판단
→ 코드 규칙: 부모는 같은 family만 — 다르거나 도구 실패면 nearest_base(추론기 값)를 부모로, family도 그것을 따름
   (nearest_base 는 선택 칸 — 없으면 같은 family 기본 설계 중 블록 수가 가장 가까운 것)
→ parts/4.0 → check_design → 저장(made_by scan) → (화면이) select_design → READY
→ 떨어지면: AI 칸 · Template 탓만이면 AI가 다시 채움(최대 2번 — SCAN_REFILLS) · 블록 수 초과 · 놓인 자리 탓(W201 로봇 조립 불가 포함)은 바로 끝
```

## 5. backend 파일 (SDD 3.1.1)

```
web/backend/
├── app.py            WebApp — 앱 만들기 · routes 등록 · frontend/out 정적 서빙 · 시작 점검(브로커 · 키 유무 · 화면)
│                     · load_rules(): robot.yaml 읽기(한 곳, 읽기 전용 — E-78: block_size_m · check.margin_mm · check.max_blocks ·
│                       finger.thickness_m · finger.width_m · grasp_depth_m(W201 손가락 자리 — E-121 ⑲) · assembly_area_half_m
│                       + 다리와 같은 MQTT 시간 mqtt.req_timeout_s · alive_s · lost_after_s, 측정값은 안 읽음)
│                     · 저장소 열기(DATABASE_URL 이 있으면 PgDesignStore — DB 를 최대 30초 기다림, 없으면 DesignStore)
│                     · d2_bridge · d2_task 의 ROS 없는 파일을 같은 저장소에서 import
├── routes/
│   ├── robot.py      /api/robot command · stop · resume · state + 그림 wrist.jpg · scan.jpg · scan_preview.ply · scan_cloud.ply → MqttClient.
│   │                 로봇 PC 끊김이면 start · scan 은 안 보냄
│   ├── designs.py    /api/designs 목록 · {id} · rules · templates · {id}/name · {id}/builds · generate(+ /{job_id} · /pick) ·
│   │                 scan/save(+ /{job_id}) · scan/recommend → DesignGenerator · DesignStore
│   ├── ws.py         /ws {"type", "data"} 한 겹 — 붙으면 들고 있는 값 먼저, 그 뒤 바뀔 때마다(밀리면 오래된 것부터 버림)
│   └── voice.py      /api/voice — 화면 마이크 → d2/hmi/voice/listen → 호스트 voice(받아 적기 gpt-4o-mini-transcribe → 안 되면 whisper-1 → 받아 적은 문장 그대로 MQTT d2/hmi/intent request_design — 의도 고르기 없음, E-96). 음성은 HMI 밖(10/13 PL)
├── mqtt_client.py    MqttClient — 브로커와 통신 한 곳: req/res(req_id · req_timeout_s, 자동 재전송 없음) · 상태 마지막 값 ·
│                     d2/web/alive 1초(LWT false) · 다리 연결 신호 3초 끊김 · serve()로 등록한 함수가 get_design · save_build 답함 ·
│                     지난 조립 진행표(retained) 비우기(E-120 10/15 12시대) (토픽 이름 · req/res 짝은 d2_bridge/bridge_codec.py 를 같이 씀)
├── design_gen.py     DesignGenerator — 설계 AI(Claude) 호출 한 곳(4-A 생성 · 4-C 스캔 채우기 scan_fill) — 주소는 중계 D2_LLM_URL
├── design_store.py   DesignStore(JSON 폴더) · PgDesignStore(PostgreSQL — .env DATABASE_URL 이 있을 때, W088) — 저장 · DB 접근 한 곳 + 형식 이름 상수 · 기본 설계 등록(d2_task 변환기 ② import, E-59)
│                     register_bases · list_designs(+ last_build) · children · get_design · builds_for · examples_for · match_scan(스캔 추천, E-97) ·
│                     templates · base_recipe · new_design_id · save_design(E-84 이름) · set_display_name(표시 이름 W197) · save_build(같은 run_id 한 번) ·
│                     save_gen_log + 로봇 get_design · save_build 답. 기본 설계는 도면에서 온 것이라 등록 때 check_design 을 부르지 않는다(로봇 PC 없이도 목록이 떠야 함)
├── prompts/          design_system.txt(설계 직접 작성 — 규칙 1 ~ 10: 7 = [Template 규칙], 8 ~ 10 = W201 조립 규칙(E-121 ⑲) + 역할 목록 + 블록마다 쓰는 칸 · 출력).
│                     숫자 자리($margin · $max_blocks · $finger_t · $finger_w · $grasp_depth · $min_bearing 등)는 design_gen 이 robot.yaml · 검사와 같은 값으로 채움.
│                     스캔 채우기 할 일은 design_gen.scan_fill 이 이 프롬프트 뒤에 붙임. 후보 3개 JSON Schema 는 design_gen._candidates_format(코드)
├── voice.py          VoiceListener — 호스트(마이크 버튼 → 시동어 → STT → d2/hmi/intent · '정지'면 d2/safety/stop/req, 점검 --check)
├── mock_robot.py     가짜 로봇 PC(MQTT 만): 다리 연결 신호 · 처음 상태 · 가짜 조립(블록마다 progress) · 정지 · 스캔(→ SCAN_REVIEW +
│                     scan_result, 추정 2개) · check_design = 진짜 검사 묶음(DesignChecker + 변환기 ①). 실행 python3 web/backend/mock_robot.py --step 1.0
├── requirements.txt  fastapi · uvicorn[standard] · paho-mqtt · pyyaml · openai(설계 AI 클라이언트 — 주소만 중계 · 음성 받아 적기) · python-dotenv · numpy<2 · psycopg[binary] — DB
└── data/             designs/ · builds/ · gen_logs/ (gitignore — JSON 폴더 저장소, compose 는 /data 로 연결). DB 데이터는 web/db/data(gitignore)
```

## 6. 다리 `d2_bridge`(로봇 PC, W127)

IRD 10.1 표 그대로. 구독 QoS는 IRD 4.1(`task/state` · `progress` · `scan_result` · `safety/state` = TRANSIENT_LOCAL depth 1, 나머지 기본 VOLATILE). `/d2/hmi/alive` 1초. 설정은 `robot.yaml` `mqtt.host` · `port` · `qos` · `req_timeout_s` · `alive_s` · `lost_after_s`(IRD 9장 — W127 PR에서 더함) — `mqtt.host`는 localhost(개발 PC용)라 로봇 PC에서는 늘 `-p mqtt_host:=<웹IP>`. 다리는 변환만 한다(판단 없음). 변환 계산은 ROS · paho 없는 파일로 떼어 `tests/`에서 pytest(CI는 `d2_bridge`를 colcon 빌드하지 않는다). 로컬 개발 PC에는 두산 패키지가 없어 정지 노드를 못 띄운다 — 정지 노드와의 확인은 로봇 PC(W129).

## 두 PC Wi-Fi 연결 — W173/W174, PR #136

화면은 웹 PC `http://localhost:8000`에서 사용한다. MQTT는 웹 PC의 Wi-Fi IPv4에 공개하되 로봇 PC IPv4만 허용한다. 로봇 PC = 박진용 PC · 웹 PC = 한석형 노트북(E-92 10/15 덧붙임) — IP는 그날 두 PC에서 `ip -4 addr show`로 확인해 아래 `<로봇IP>` · `<웹IP>`에 넣는다(10/15에 웹 PC가 바뀌었다가 돌아와 예전 값과 다를 수 있다). Wi-Fi의 다른 기기를 자동 허용하지 않는다. IP가 바뀌면 두 실제 IP를 확인하고 아래 설정을 다시 적용한다.

Docker의 공개 포트는 ufw만으로 제한하지 않고 DNAT 전 목적지를 검사하는 DOCKER-USER 규칙을 사용한다. 컨테이너 내부 web→mosquitto 통신은 이 호스트 공개 포트 규칙에 포함되지 않는다. IPv4 주소에만 바인딩하여 IPv6 전체 공개를 하지 않는다.

브로커가 정지한 상태에서 먼저 방화벽을 적용한 뒤 기동한다:

```bash
sudo bash tools/mqtt_access.sh apply <로봇IP> <웹IP>
sudo bash tools/mqtt_access.sh check <로봇IP> <웹IP>
D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml up -d
```

위 방화벽 규칙은 ORIGINAL 요청 방향만 검사한다. 응답도 원본 목적지와 일치하므로 방향을 제한하지 않으면 허용 로봇의 SYN-ACK까지 거부된다. 첫 번째 로봇 허용, 두 번째 다른 출발지 거부와 방향 설정을 검사한다. 방화벽 검사 실패 시 브로커를 공개하지 않는다. 장치/토픽 확인과 로봇 이동을 같은 시험으로 집계하지 않는다.

재부팅 자동 적용은 실제 설치 구성 하나를 사용한다. 브로커 정지 상태에서 저장소 루트에서 실행한다:

```bash
sudo bash tools/install_mqtt_access.sh <로봇IP> <웹IP>
```

설치 도구가 저장소의 mqtt_access.sh를 `/usr/local/libexec/d2-mqtt-access.sh`로 복사하고 `/etc/d2-mqtt-access.env`에 두 IPv4를 기록한다. 아래 내용으로 `/etc/systemd/system/d2-mqtt-access.service`를 만들고 daemon-reload, enable, restart, 규칙 check를 실행한다. 기존 브로커가 실행 중이면 설치를 거절한다.

```ini
[Unit]
Description=D2 MQTT access restriction
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target
PartOf=docker.service

[Service]
Type=oneshot
EnvironmentFile=/etc/d2-mqtt-access.env
ExecStart=/bin/bash /usr/local/libexec/d2-mqtt-access.sh apply ${D2_ROBOT_IPV4} ${D2_WEB_IPV4}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

실제 재부팅 후 웹 PC에서 다음으로 확인한다. enabled/active와 실제 규칙 출력까지 저장한다. 브로커는 자동 기동하지 않으므로 check 통과 뒤 위 compose 기동 명령을 실행한다.

```bash
systemctl is-enabled d2-mqtt-access.service
systemctl is-active d2-mqtt-access.service
sudo bash /usr/local/libexec/d2-mqtt-access.sh check <로봇IP> <웹IP>
sudo iptables -w -S DOCKER-USER
```

Wi-Fi IP 변경 시 브로커를 정지하고 설치 도구를 새 두 IP로 다시 실행한다. 설치가 env를 갱신하고 서비스를 재시작한다. 새 웹 IP로 D2_WEB_IPV4를 지정해 compose를 다시 기동한다. 이 정책은 IP 변경을 자동 감지하거나 다른 기기를 자동 허용하지 않는다.

검증: 로봇 PC는 1883 연결 성공, 비허용 PC는 1883 연결 실패, 외부 PC는 8000 연결 실패, 웹 PC localhost:8000 성공을 각각 실제로 확인한다. 방화벽 적용 전 수행한 과거 LAN 성공 결과를 제한 적용 후 검증으로 대신 쓰지 않는다.

외부 PC 시험 명령(로봇 PC와 다른 PC에서 각각 같은 명령 실행):

```bash
python3 tools/check_web_ports.py <웹IP>
```

명령은 TCP 연결 후 닫기만 하며 MQTT 메시지/로봇 명령을 보내지 않는다. `source_ip`와 `connected`를 기록한다. 로봇 PC 1883 = true, 다른 PC 1883 = false · 8000 = false가 필요하다. 웹 PC 자신이나 내부 컨테이너의 접속은 외부 PC 시험으로 대체하지 않는다.

## DB 켜기 · 끄기 (W088 — PostgreSQL `db` 컨테이너)

저장소 함수 이름 · 반환 모양은 JSON 폴더와 같다(`design_store.PgDesignStore`, 표 = `web/db/init/001_schema.sql` — 정본 PM 설계_DB 부록 A + 형식 4.0 + `gen_logs.extra`).
- 켜기: `web/.env`에 `COMPOSE_PROFILES=db` · `POSTGRES_PASSWORD=<비밀번호>` · `DATABASE_URL=postgresql://d2:<비밀번호>@db:5432/d2` → `D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml up -d --build`(이미지에 psycopg — 처음 한 번 `--build`). 첫 시작 때 빈 `web/db/data/`에 표를 만들고, web 이 기본 설계 4개를 다시 등록한다. web 은 DB 를 최대 30초 기다린다(`DB_TRIES`).
- **표시 이름(W197, 10/14 PL):** 설계 기록 칸 `display_name`(DB 칸 같은 이름, 40자 이하 · 한 줄) — `PUT /api/designs/{id}/name {name}`(빈 글자 · null = 지움). 화면 이름은 `names.ts designName()` 한 곳에서 '이름 · 벤치 V001'(없으면 '벤치 V001'), 설계 3D 제목 옆 [이름 고치기]. design_id · 레시피 · part_id 는 그대로, 기본 설계를 다시 등록해도 이름은 남음(저장한 설계에서 고칠 수 있는 칸은 이것뿐 — E-55 예외, E-115). 바꿀 때마다(지움 포함) 생성 기록 gen_logs 에 kind `rename` 한 줄(design_id · 바꾼 때 · 이전 · 새 이름 — DB 는 extra 칸, model · prompt_sha = '-')
- 어느 저장소인지는 화면 위 연결 점 옆 글 '저장 DB' / '저장 파일'(`GET /api/designs/rules` 의 `store`, 10/14 PL)와 backend 로그 '[웹] 저장소 = …'. `DATABASE_URL` 이 있는데 DB 에 못 붙으면 JSON 으로 자동으로 돌아가지 않고 web 이 서서 다시 켜진다(기록이 두 곳으로 갈라지지 않게).
- 끄기(JSON 폴더로 되돌리기 — 시연 중 DB 가 말썽일 때): `DATABASE_URL=` 비우고 `D2_WEB_IPV4=<웹IP> WEB_UID=$(id -u) WEB_GID=$(id -g) docker compose -f web/compose.yaml up -d web`. DB 데이터(`web/db/data/` — gitignore, 웹 PC 로컬 폴더)는 그대로 남는다. JSON 폴더와 DB 사이에 설계를 옮기지 않는다(옛 3.0 설계는 E-111 로 버림).
- `recipe` 칸만 `json` 형(넣은 글자 그대로 — `-0.0` 이 남아 `recipe_sha256` 이 맞음, 설계_DB 8절 A). DB 끊김 = `OSError`(save_build 일시 실패 → 로봇이 다시 보냄), 키 겹침 · 제약 · 외래 키 = `ValueError`(409 · save_build 영구 거절).
- 시험: `D2_TEST_DATABASE_URL=<빈 시험 DB> python3 -m pytest tests/test_design_store_pg.py`(표를 비운다 — 시연 DB 에 쓰지 않는다). 없으면 건너뜀(CI).
