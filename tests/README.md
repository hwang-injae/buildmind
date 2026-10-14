# tests — ROS 없이 도는 자동 시험

저장소 루트에서 `python3 -m pytest tests -q` 로 돈다(개인 venv 면 `~/venvs/<이름>/bin/python -m pytest tests -q`). 도구는 `pip install -r tests/requirements.txt`(pytest · pyyaml · ruff). CI(`.github/workflows/ci.yml`)가 PR(main 대상)과 main push 마다 같은 것과 `src/d2_task/test` · `src/d2_vision/test`(ROS 없이 pytest)를 돌린다 — 토큰(Claude API)은 쓰지 않는다.

| 파일 | 무엇을 지키나 |
|---|---|
| `test_pr_check_rules.py` | `.github/scripts/pr_check.py` 정규식 — 키 모양 · 문서 이름 · 비밀 파일 · 로봇 이동 코드 · 정지 설정 단어 |
| `test_pr_verdict_rules.py` | `.github/scripts/pr_verdict.py` 자동 승인 규칙 — 체크 안 된 'PL 확인' 줄이 있거나 IRD 내용 · `d2_interfaces`를 바꿨으면 자동 승인하지 않음(10/7) |
| `test_docs_rules.py` | `docs/` 문서 이름 규칙(팀 규칙 9) · 저장소 `.md` 의 상대 링크가 깨지지 않음(깨진 것은 모두 모아 보여 줌) |
| `test_robot_yaml.py` | `src/d2_robot/d2_bringup/config/robot.yaml` 필수 키 · 형식 · 공급 칸 6개(코드 경로만 남은 옛 키 — 10/13 E-103으로 안 씀) · 잡기 폭 순서(닫기 < 폭 < 놓기 열림 ≤ 집기 열림) · 개인 경로 없음 · 같은 키 두 번 없음 |
| `test_interfaces.py` | `d2_interfaces` 의 `.srv`/`.action` 구분선 수 · 칸 줄 형식 · 약속한 칸 이름(IRD 5장) · CMakeLists 등록 |
| `test_motion_math.py` | `d2_motion.motion_math` — 회전 왕복 · TCP 자세 · 잡기 이름 · robot.yaml 공급 칸 자세와 잡기가 맞음 · RG2 손가락 끝 높이 · 집기·놓기 TCP · 레시피(실측 두께로 쌓기) |
| `test_recipe_manager.py` | `src/recipe_manager` + `d2_task.recipe_builder.RecipeBuilder`(10/8 E-69) — 기본 설계 4종을 DXF에서 다시 만들면 저장된 `_recipe.json`(구조 `recipe/4.0`) · `_placements.csv`(조립 방법, schema `placements/4.0` — 10/14 E-111 C6)와 글자까지 같음 · `cads/`엔 계획 파일 없음 · `recipes/`엔 모형마다 2개만(`_structure.json` 없음) · CSV 칸 · 모든 줄 `recipe_sha256` = 구조 객체를 키 정렬 · 공백 없는 JSON(UTF-8)으로 만든 sha256(값 고정 — 계산 방법이 바뀌면 깨짐) · 블록 이름 꼴(역할 + 옵션 0~1) · 번호 순서 · 겹침 · 책상 아래 · 뜬 블록 · 순서 · 단계 · 잡기 상태 · 빠진 CAD 속성 거부 · 옛 잡기 이름(SIDE_25 · END_75) 변환. 변환기 ① 시험은 `src/d2_task/test/test_parts_to_recipe.py`(W110 #99) · `test_check_design_service.py`(#100)에 있다 |
| `test_no_secrets.py` | 텍스트 파일에 키 모양 없음 · `.env`/`.pem` 등 비밀 파일 없음 · `src/` 에 `/home/` 경로 없음 |
| `test_shared_tables.py` | 여러 파트가 따로 든 같은 표 · 규칙(설계 잡기 6개 · 방향 코드 · 부품 이름 규칙 · 레시피 지문 · 형식 앞자리 비교)이 정본 `d2_task.recipe_document` · `recipe_to_parts`와 같음 — 패키지 경계로 가져다 쓸 수 없는 복사본(`d2_motion` · `d2_vision` · 화면 TypeScript · `robot.yaml`)을 본다(10/14 PL E-111 C1 ~ C4) |
| `test_bridge_codec.py` | 다리 변환(`src/d2_bridge/d2_bridge/bridge_codec.py`)이 IRD 10장대로 토픽 이름 · 요청 · 응답 JSON을 만들고 틀린 요청을 고치지 않고 거절함 — ROS · paho 없이(CI는 `d2_bridge`를 colcon 빌드하지 않음) |
| `test_web_backend.py` | 웹 backend(`mqtt_client` · `design_store` · `routes`)가 IRD 6 · 10장대로 — `…/req` + `req_id` 짝 · 시간 초과 · 다리 연결 신호 3초 끊김 · 로봇 PC 끊기면 출발 · 스캔 막음 · 기본 설계 4개를 `design/4.0`으로 등록 · 옛 형식 거절 · 같은 `run_id`는 한 번만 저장(가짜 paho) |
| `test_design_gen.py` | AI 설계 생성(`web/backend/design_gen.py`)이 SDD 6.6 · IRD 8.3 · E-72 · E-84대로 — 부모 = 사람이 고른 설계 · 도구는 같은 Template의 다른 설계만 · 합격 후보만 저장 · 모자라면 검사 이유로 더(최대 2번) · `OUT_OF_SCOPE` · 중계 꺼짐 · 사용 한도 · 스캔 [저장](가짜 AI 클라이언트 + 진짜 검사 묶음) |
| `test_claude_bridge.py` | 설계 AI 중계 `web/claude_bridge.py`(10/13 E-99) — `claude` 실행 없이 가짜 실행 함수로 요청 → `claude` 인자 · 응답 모양, 진짜 `design_gen`을 붙여 도구 읽기 → 후보 → 검사까지 |
| `test_scan_recommend.py` | 스캔 '추천 설계'(`DesignStore.match_scan`, E-97)가 비전 일치율(V-48 `match_rate`)과 같게 고름 |
| `test_mqtt_access.py` | `tools/mqtt_access.sh` 방화벽 규칙 만들기 · 확인(웹 PC 브로커를 로봇 PC만 받게, #136) — 가짜 iptables로, 실제 차단은 실기 확인 |
| `test_w144_recorder.py` | `tools/w144_capture/placement_recorder.py` — 배치 뒤 observe 촬영을 더하는 것만(이동 · 촬영은 가짜, 실기 확인 아님) |

`conftest.py` 가 저장소 루트(`ROOT`)·`robot.yaml`(`robot_cfg`)·`pr_check.py`(`pr_check_mod`)·저장소 파일 목록(`repo_files`)을 fixture 로 준다. `src/d2_robot/d2_motion` 을 `sys.path` 에 넣어 `d2_motion.motion_math` 를 ROS 없이 import 한다.

## 새 시험을 더할 때

- **ROS 없는 계산·설정·문서 규칙**은 여기 `tests/` 에, 파일 이름은 `test_<무엇>.py`. 파일 맨 위 docstring 에 무엇을 지키는지 한 줄.
- **ROS 노드 시험**은 `src/<패키지>/test/` 에 둔다(`colcon test`). rclpy 없이 도는 `d2_task` · `d2_vision` 시험은 CI가 pytest로 바로 돌린다(`ci.yml`). 그 패키지 `setup.py` 에 `tests_require=['pytest']` 가 있어야 colcon 이 pytest 로 돌린다(없으면 `setup.py test` 로 떨어져 시험이 돌지 않는다). CI 는 **두산 의존이 없는 패키지만** colcon build·test 한다(`ci.yml` 의 `CI_PKGS`) — 두산·OnRobot 패키지가 필요한 `d2_bringup`·`d2_motion`·`d2_gripper`·`d2_safety` 는 실기 PC에서 빌드한다.
- 숫자·좌표는 지어내지 않고 `robot_cfg` fixture 로 `robot.yaml` 을 읽는다. 관계(순서·합·일치)를 본다.
- 키 모양 문자열이 필요하면 파일에 쓰지 않고 실행 때 이어 붙인다(`'sk-' + 'proj-' + 'A' * 30`) — PR 검사가 파일 안의 키 모양을 막는다.
- 저장소 루트에 `pytest.ini`·`pyproject.toml` 을 두지 않는다 — `colcon test` 가 그것을 rootdir 로 잡는다. 실행은 늘 `python3 -m pytest tests`.
