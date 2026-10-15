# 로봇 시스템

> 10/16 최종 정리(main c0d5c71 기준).

M0609 팔·RG2 그리퍼·손목카메라가 장착된 로봇 시스템의 구성·동작·정지를 모은다.
`d2_robot/`은 소스 폴더 묶음이며 별도 ROS 패키지는 아니다. 각 패키지 이름·노드·통신 계약은 유지한다.

| 패키지 | 책임 |
|---|---|
| [d2_bringup](d2_bringup/) | 모델·장착 관계·TCP·공통 설정·launch |
| [d2_motion](d2_motion/) | 집기·놓기, MoveIt2 계획·실행, 충돌 장면 관리 |
| [d2_gripper](d2_gripper/) | RG2 제어·파지 확인 |
| [d2_safety](d2_safety/) | 정지 판단·서기 궤적·잠금·재개 |

카메라의 장착 모델·좌표는 bringup에 둔다. 관측 해석은 [d2_vision](../d2_vision/),
조립 순서·완료 판단과 설계 ↔ 레시피 변환기 ① · ②는 [d2_task](../d2_task/)(10/8 E-58), 공통 메시지는 [d2_interfaces](../d2_interfaces/),
CAD → 레시피 도구는 [recipe_manager](../recipe_manager/)에서 담당한다.

공통 설정 정본은 [robot.yaml](d2_bringup/config/robot.yaml)이다. 실행 시에는 계속
`get_package_share_directory('d2_bringup')`에서 설치된 설정·모델을 읽는다.
팔 명령은 집기·놓기, 실제 Scene 변경은 장면 관리, 정지 판단은 정지 노드가 각각 담당한다.

## 빌드 · 실행

저장소 루트에서 ROS 2와 두산 워크스페이스 환경을 불러온 뒤 실행한다([src/README](../README.md) '빌드 · 시험').

```bash
colcon build --symlink-install --packages-select d2_bringup d2_motion d2_gripper d2_safety
source install/setup.bash
```

main을 받은 뒤에는 바뀐 패키지를 전부 다시 빌드한다 — `robot.yaml` · 설치 레시피를 바꾼 커밋이면 `d2_bringup`도 꼭 넣는다(설치 폴더에서 읽으므로 빌드하지 않으면 옛 값이 남음). 순서는 [src/README](../README.md) '빌드 · 시험'.

launch는 **패키지 이름으로** 실행한다(`src/…` 경로로 실행하면 `__pycache__`가 생긴다). 터미널 두 개 — 브링업을 먼저 띄우고, 다 뜬 뒤 노드 4개를 띄운다.

```bash
# 터미널 1 — 브링업: 두산 드라이버(dsr_hardware2) · 컨트롤러 · 그리퍼 드라이버 · MoveIt2 move_group · RViz
ros2 launch d2_bringup real_moveit.launch.py                                   # 가상(두산 에뮬레이터 · 가상 그리퍼)
ros2 launch d2_bringup real_moveit.launch.py mode:=real host:=192.168.1.100    # 실기
# 터미널 2 — 노드 4개: safety_stop · gripper · scene_manager · pick_place
ros2 launch d2_bringup robot_nodes.launch.py                # 가상
ros2 launch d2_bringup robot_nodes.launch.py mode:=real     # 실기
```

`real_moveit.launch.py` 인자(코드 `DeclareLaunchArgument` 그대로):

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `mode` | `virtual` | `virtual`(에뮬레이터 + 가상 그리퍼) · `real`(실제 컨트롤러 + OnRobot RG2 드라이버 Modbus). 둘 다 그리퍼 명령은 `/onrobot/sendCommand` |
| `host` | `127.0.0.1` | 로봇 컨트롤러 IP — 실기는 `192.168.1.100` |
| `port` | `12345` | 컨트롤러 포트(xacro · 에뮬레이터에 넘김) |
| `rt_host` | `192.168.137.50` | 두산 드라이버 · 에뮬레이터에 그대로 넘기는 `rt_host` |
| `tcp_z` | `config/tcp.json`에서 계산 | `rg2_base_link` → 손가락 끝(m). 보통 주지 않는다 — 모델 TCP는 `tcp.json`(제어기 활성 TCP)에서 만든다 |
| `gripper_ip` | `192.168.1.1` | 실기 RG2(Modbus) IP |
| `rviz` | `true` | RViz 띄우기 |

`robot_nodes.launch.py` 인자:

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `mode` | `virtual` | `real`이 아니면 그리퍼 노드를 가상 그리퍼용(`virtual` 파라미터 true)으로 켠다 |
| `recipe` | 빈 값 | 장면 관리가 읽을 레시피 파일 — 비우면 설치된 레시피 전부(하나씩), 여럿이면 쉼표로 = 세트 배치(`run_recipe`와 같은 목록) |

- 두 launch에는 카메라 · 작업 관리자(`task`) · 손목 비전(`wrist_block` · `scan_live_preview`) · 다리(`bridge`)가 없다 — 따로 `ros2 run`한다(실행 순서 SDD 3.5). 수업 브링업(`m0609_rg2_bringup bringup.launch.py`)은 MoveIt을 띄우지 않으므로 이 브링업과 같이 띄우지 않는다.
- 브링업이 떴는지는 로그 `You can start planning now!`, 노드 4개는 `[집기·놓기] 준비 끝`으로 본다. 브링업을 다시 켜면 노드 4개도 다시 켠다.
- 가상 에뮬레이터 실행 · 종료 때 컨테이너 삭제 처리는 [저장소 규칙](../../AGENTS.md) 3장을 먼저 확인한다(지우기 전 `docker ps -a`).
