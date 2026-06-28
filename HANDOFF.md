# Husky + Franka 모바일 매니퓰레이터 — HANDOFF

_최종 업데이트: 2026-05-29_  
_환경: Isaac Sim 5.1 / ROS2 Jazzy / Ubuntu 24.04 ARM64_

## 1. 최종 목표

Husky 위에 Franka Panda arm을 결합한 모바일 매니퓰레이터를 만들고, 최종적으로는 **라이다 PointCloud2로 감지한 물체 위치를 추정해서 물체를 집는 것**이 목표다.

현재는 라이다 perception 전 단계까지 진행했다. 즉:

1. Husky + Franka를 단일 articulation으로 결합했다.
2. Franka pick and place 예제 구조를 분석했다.
3. 결합 로봇(`/husky`)에서 cube ground-truth 위치를 사용해 pick/place가 되도록 만들었다.
4. 움직이는 Husky에서 grasp를 시작하는 테스트 스크립트까지 준비했다.
5. 다음 단계는 **cube ground-truth 대신 lidar PointCloud에서 cube 상대 위치를 추정하는 perception layer**를 붙이는 것이다.

## 2. 머신/환경 정보

- 머신: NVIDIA GB10 / ARM64(aarch64) / Ubuntu 24.04 Noble
- Isaac Sim 5.1 경로:
  - `/home/user/Desktop/isaac-sim-5.1`
  - Isaac Sim 번들 Python: `/home/user/Desktop/isaac-sim-5.1/python.sh`
- 프로젝트 폴더:
  - `/home/user/Desktop/260527 KMS`
- ROS2:
  - ROS2 Jazzy
  - apt ROS2는 Python 3.12 기준
- 중요 Python 주의:
  - 기본 `python3`는 conda/miniconda 3.13일 수 있음
  - ROS2 노드는 반드시 `/usr/bin/python3`로 실행
  - `run_controller.sh`, `run_rviz.sh`는 이 문제를 피하기 위한 wrapper

ROS 실행 시:

```bash
source /opt/ros/jazzy/setup.bash
```

또는 zsh에서 직접 작업할 때는:

```zsh
source /opt/ros/jazzy/setup.zsh
```

## 3. 핵심 파일 목록

프로젝트 루트: `/home/user/Desktop/260527 KMS`

| 파일 | 역할 |
|---|---|
| `HANDOFF.md` | 현재 핸드오프 문서 |
| `husky_test.usdz` | 원본 Husky + lidar 씬. 중요 백업 파일 |
| `husky_franka.usd` | Franka 결합 완료 씬. 현재 주 작업 씬 |
| `mount_franka.py` | Husky에 Franka를 결합하는 Script Editor용 스크립트 |
| `arm_control_test.py` | 결합된 `/husky` articulation에서 팔 관절 직접 제어 테스트 |
| `reach_test_husky.py` | RMPflow가 결합 articulation에서 cube 위로 reach 가능한지 확인했던 Phase A 테스트 |
| `pick_place_husky.py` | 현재 성공한 ground-truth cube pick/place 스크립트. Lula IK direct 사용 |
| `pick_place_husky_rmp.py` | RMPflow 실패/느림 비교 영상용 스크립트 |
| `pick_place_husky_mobile.py` | 움직이는 Husky에서 cube가 reach 범위에 들어오면 pick/place 시작하는 테스트 스크립트 |
| `lidar_cube_detector.py` | `/point_cloud` + `/tf`에서 cube/stand cluster 후보를 잡아 `/detected_cube`와 `/tmp/lidar_cube_latest.json`으로 출력 |
| `pick_place_husky_lidar.py` | `cube.get_world_pose()` 대신 `/tmp/lidar_cube_latest.json`의 lidar 추정 target으로 pick/place 시도 |
| `diagnose_poses.py` | STOP 상태에서 Husky/Franka 주요 prim pose 진단 |
| `diagnose_floor.py` | SimpleRoom floor 높이와 top-level prim pose 진단 |
| `lift_floor_to_zero.py` | SimpleRoom floor를 z=0 기준으로 올리는 스크립트 |
| `husky_controller.py` | OpenCV 가상 조이스틱. `/cmd_vel` publish |
| `run_controller.sh` | ROS source 후 `/usr/bin/python3 husky_controller.py` 실행 |
| `run_rviz.sh` | RViz 실행 wrapper |
| `run_lidar_detector.sh` | ROS source 후 `/usr/bin/python3 lidar_cube_detector.py` 실행 |
| `run_pointcloud_to_scan.sh` | `/point_cloud`를 `/scan` LaserScan으로 변환하는 SLAM 준비 wrapper |
| `run_slam_toolbox_husky.sh` | Husky lidar SLAM용 `slam_toolbox` 실행 wrapper |
| `slam_toolbox_husky_params.yaml` | 첫 SLAM 실험용 slam_toolbox parameter file (`odom_frame=world`, `base_frame=sensor`) |
| `husky_lidar.rviz` | RViz 설정. Fixed Frame=`world`, `/point_cloud` 표시 |

## 4. 현재 씬 상태

작업 씬은 `husky_franka.usd`.

현재까지 처리된 중요한 씬 수정:

1. Franka가 `/husky/panda` 아래에 들어가 있다.
2. 전체 로봇은 **단일 articulation root `/husky`**로 동작한다.
3. Franka 독립 articulation root/rootJoint 문제를 제거했다.
4. `panda_mount_joint`가 `/husky/base_link`와 `/husky/panda/panda_link0`을 FixedJoint로 연결한다.
5. SimpleRoom의 실제 floor를 z=0 기준으로 lift했다.
6. 시각적으로 보이는 실내 floor surface는 `/SimpleRoom/Towel_Room01_floor_bottom_218/Towel_Room01_floor_bottom`이고, z는 약 `+0.0458`.

주의:

- `/SimpleRoom/Floor/SM_Template_Map_Floor`는 z=0.
- 실제 cube/stand를 올려야 하는 walking surface는 z=0이 아니라 약 `+0.0458`.
- 그래서 pick/place 스크립트는 `FLOOR_PRIM = "/SimpleRoom/Towel_Room01_floor_bottom_218/Towel_Room01_floor_bottom"`에서 z를 읽어 cube/stand 높이를 결정한다.

## 5. Franka 결합 방식 요약

`mount_franka.py`가 한 일:

1. Franka USD를 `/husky/panda`로 reference.
2. Franka의 원래 world 고정 `rootJoint`를 비활성화.
3. `/husky/panda`의 독립 ArticulationRootAPI 제거.
4. `/husky/base_link`와 `/husky/panda/panda_link0` 사이에 FixedJoint 생성.
5. 최종적으로 `husky_franka.usd`로 저장.

중요한 결론:

- 예제의 Franka는 `/World/Franka` 자체가 articulation root.
- 현재 로봇은 `/husky`가 articulation root이고 Franka는 그 안의 일부.
- 따라서 `SingleManipulator("/husky/panda")` 같은 접근은 맞지 않는다.
- 현재는 `SingleArticulation("/husky")`를 잡고, `joint_indices`로 Franka arm 관절 7개만 골라 명령한다.

현재 `/husky` articulation DOF:

- 총 13 DOF
- 바퀴 4개
- Franka arm 7개
- gripper finger 2개

실제 dof order는 로딩 시 다음처럼 나왔었다:

```text
['panda_joint3', 'panda_joint4', 'panda_joint2', 'panda_joint5',
 'panda_joint1', 'panda_joint6', 'panda_joint7',
 'front_left_wheel_joint', 'front_right_wheel_joint',
 'rear_left_wheel_joint', 'rear_right_wheel_joint',
 'panda_finger_joint1', 'panda_finger_joint2']
```

그래서 arm index는:

```text
panda_joint1 -> 4
panda_joint2 -> 2
panda_joint3 -> 0
panda_joint4 -> 1
panda_joint5 -> 3
panda_joint6 -> 5
panda_joint7 -> 6
```

코드에서는 항상 `robot.get_dof_index(name)`로 동적으로 구한다.

## 6. Franka pick and place 예제 분석 요약

원본 예제 경로:

```text
/home/user/Desktop/isaac-sim-5.1/standalone_examples/api/isaacsim.robot.manipulators/franka_pick_up.py
```

중요한 import 계층:

```text
franka_pick_up.py
  -> World
  -> SingleManipulator
  -> ParallelGripper
  -> PickPlaceController
      -> Base PickPlaceController 10-phase state machine
      -> RMPFlowController
          -> isaacsim.robot_motion.motion_generation
```

예제 동작 핵심:

- 실행되는 진입점은 `franka_pick_up.py` 하나.
- 하지만 실제 동작은 여러 Isaac Sim library class가 import되어 구성된다.
- pick 위치는 `cube.get_local_pose()[0]`로 USD prim ground-truth를 읽는다.
- place 위치는 `np.array([-0.3, -0.3, 0.0515/2.0])` 같은 hard-coded 상수다.
- perception은 없다.

원본 예제의 핵심 루프:

```python
my_world.step(render=True)
actions = my_controller.forward(
    picking_position=cube.get_local_pose()[0],
    placing_position=np.array([-0.3, -0.3, 0.0515 / 2.0]),
    current_joint_positions=my_franka.get_joint_positions(),
    end_effector_offset=np.array([0, 0.005, 0]),
)
articulation_controller.apply_action(actions)
```

PickPlaceController는 10-phase 상태머신:

```text
0: cube 위 hover
1: cube로 하강
2: settle 대기
3: gripper close
4: 들어올림
5: place xy로 이동
6: place z로 하강
7: gripper open
8: 다시 위로 상승
9: 복귀
```

상태머신은 "손이 어디로 가야 하는지"를 정하고, RMPflow/IK 같은 cspace controller가 "그 위치에 가려면 관절각이 무엇인지"를 계산한다.

## 7. RMPflow와 Lula IK direct 정리

둘 다 Isaac Sim의 `isaacsim.robot_motion.motion_generation` 쪽 기능이다.

정확한 표현:

- RMPflow: motion policy
- Lula IK: inverse kinematics solver
- 둘 다 Isaac Sim Motion Generation API에서 제공된다.

발표용 표현:

```text
기본 Franka 예제는 RMPflow motion policy를 사용한다.
결합 로봇에서는 RMPflow 응답이 너무 느려 phase 시간 안에 cube에 도달하지 못했다.
그래서 PickPlaceController 상태머신은 유지하고, motion generation 부분만 Lula IK solver 기반 direct IK 방식으로 교체했다.
```

RMPflow:

- 매 step 목표 방향으로 조금씩 움직이는 reactive motion policy.
- 부드럽고 collision avoidance/가속도 제한에 강점.
- 이번 결합 로봇에서는 너무 느려서 cube에 도달하기 전에 phase가 넘어갔다.

Lula IK direct:

- 원하는 end-effector pose를 만족하는 7개 arm joint angle을 직접 계산.
- 계산된 joint target을 PhysX PD drive에 넣어 빠르게 도달.
- collision avoidance는 없다.
- 현재 단순 stand/cube pick-place demo에는 충분히 잘 동작했다.

중요한 API convention 차이:

- RMPflow target은 `panda_link0` local frame 기준으로 넣는 방식으로 사용했다.
- LulaKinematicsSolver는 target position/orientation을 **world frame 기준**으로 기대한다.
- 처음에 local 좌표를 Lula IK에 넣어 100% IK fail이 났고, world 좌표로 바꿔 해결했다.

## 8. 현재 성공한 pick/place 방식

성공 스크립트:

```text
pick_place_husky.py
```

실행 방법:

1. Isaac Sim에서 `husky_franka.usd` 열기.
2. STOP 상태.
3. Script Editor에서:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky.py').read())
```

4. PLAY.

동작:

1. Husky 앞에 회색 stand 생성.
2. stand 위에 파란 cube 생성.
3. cube world pose를 ground-truth로 읽음.
4. PickPlaceController 10-phase 상태머신 실행.
5. cspace controller는 RMPflow가 아니라 `_IKCSpace`.
6. `_IKCSpace`가 `LulaKinematicsSolver.compute_inverse_kinematics(...)` 호출.
7. 결과 7개 joint target을 `/husky` articulation의 arm joint index에만 적용.
8. gripper close/open은 `ParallelGripper`가 finger joint에 적용.
9. cube를 Husky top plate 위로 옮기는 데 성공.

성공 로그 예:

```text
[B] step 240 phase=3 EE_z=+0.472 cube_z=+0.420
[B] step 270 phase=4 EE_z=+0.485 cube_z=+0.440
[B] step 300 phase=5 EE_z=+0.588 cube_z=+0.544
[B] step 330 phase=5 EE_z=+0.617 cube_z=+0.572
[B] PICK & PLACE DONE.
```

성공 판단:

- phase 4 이후 `cube_z`가 올라가면 gripper가 cube를 잡은 것이다.
- 이전 실패에서는 `cube_z`가 계속 고정이었다.

## 9. 현재 성공 스크립트의 주요 설계

`pick_place_husky.py` 핵심 상수:

```python
STAND_HEIGHT = 0.35
STAND_XY = 0.25
STAND_FORWARD = 0.70
PLACE_OFFSET_TOPLATE_LOCAL = Gf.Vec3d(0.30, 0.0, 0.10)
TOOL_CENTER_OFFSET_Z = 0.103
```

중요한 의미:

- `STAND_HEIGHT=0.35`: cube를 Franka reach 가능한 높이로 올리기 위해 stand 사용.
- `STAND_FORWARD=0.70`: Husky가 떨어질 때 stand와 간섭하지 않도록 앞쪽으로 배치.
- `PLACE_OFFSET_TOPLATE_LOCAL=(0.30, 0, 0.10)`: top_plate local 기준. +x가 Husky 전방, +z가 위쪽.
- `TOOL_CENTER_OFFSET_Z=0.103`: `panda_hand` origin과 실제 grasp point/tool_center 사이 보정.

왜 `TOOL_CENTER_OFFSET_Z`가 필요한가:

- Lula IK의 frame은 `panda_hand`.
- 하지만 실제 cube를 잡는 위치는 `panda_hand/tool_center`, 즉 `panda_hand` local +z 방향 0.103m 지점.
- 그래서 `PickPlaceController.forward(..., end_effector_offset=np.array([0,0,0.103]))`를 넣었다.

그리퍼 설정:

```python
joint_opened_positions=np.array([0.04, 0.04])
joint_closed_positions=np.array([0.022, 0.022])
action_deltas=None
```

cube가 5cm라 finger 한쪽을 0.022 근처로 닫도록 맞췄다.

## 10. RMPflow 실패 비교 영상용 스크립트

파일:

```text
pick_place_husky_rmp.py
```

목적:

- 발표용 Before 영상.
- RMPflow를 사용하면 EE가 너무 천천히 움직여 phase 시간 안에 cube에 도달하지 못하는 것을 보여준다.

실행:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_rmp.py').read())
```

예상 결과:

- EE가 천천히 cube 쪽으로 움직인다.
- cube를 잡지 못한다.
- `cube_z`가 끝까지 거의 변하지 않는다.

이 스크립트는 성공용이 아니라 비교/발표용이다.

## 11. 움직이는 Husky 테스트 스크립트

파일:

```text
pick_place_husky_mobile.py
```

목적:

- Husky를 조이스틱으로 움직여 cube에 접근한다.
- cube가 arm reach 범위에 들어오면 자동으로 pick/place를 시작한다.
- 아직 lidar perception은 아니다. cube 위치는 여전히 ground-truth다.
- 다만 Husky가 움직이는 상황에서 base pose와 place pose가 매 step 갱신되는지 검증한다.

실행:

1. `husky_franka.usd` 열기.
2. STOP.
3. Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_mobile.py').read())
```

4. PLAY.
5. OpenCV joystick으로 Husky를 stand/cube 쪽으로 천천히 전진.

콘솔 로그:

```text
[M] WAIT reach_xy=...
[M] START pick/place. reach_xy=...
[M-IK] OK call#1 ...
[M] step ... phase=... reach_xy=... EE_z=... cube_z=...
[M] MOBILE PICK & PLACE DONE.
```

중요:

- `WAIT`는 아직 cube가 reach band 밖이라는 뜻.
- `START`가 뜨면 자동 pick/place 시작.
- grasp 중에는 Husky를 멈추는 것이 좋다. 계속 움직이면 target도 움직이고 arm이 흔들릴 수 있다.

현재 reach trigger:

```python
REACH_MIN_XY = 0.45
REACH_MAX_XY = 0.72
```

즉 `panda_link0`와 cube의 world xy 거리 기준으로 0.45~0.72m에 들어오면 grasp 시작.

## 12. joystick controller 실행

파일:

```text
husky_controller.py
run_controller.sh
```

실행:

```bash
cd "/home/user/Desktop/260527 KMS"
DISPLAY=:0.0 ./run_controller.sh
```

백그라운드 실행 예:

```bash
cd "/home/user/Desktop/260527 KMS"
DISPLAY=:0.0 ./run_controller.sh > /tmp/husky_controller.log 2>&1 &
```

확인:

```bash
pgrep -af "husky_controller.py"
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list -t | grep /cmd_vel
```

정상 상태:

```text
/husky_virtual_joystick
/cmd_vel [geometry_msgs/msg/Twist]
```

주의:

- controller 창을 닫으면 프로세스가 종료된다.
- 다시 켜면 OpenCV joystick 창이 뜬다.
- 창이 안 보이면 다른 창 뒤에 숨어 있을 수 있다.

## 13. STOP/PLAY 관련 중요한 주의점

Script Editor에서 다음처럼 실행하고 PLAY하면 정상:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_mobile.py').read())
```

하지만 한 번 실행 후:

```text
STOP -> 그냥 PLAY
```

만 하면 warning이 뜰 수 있다:

```text
Physics Simulation View is not created yet in order to use get_joint_positions
```

이유:

- Python callback object는 살아있다.
- `mobile_state["init"]`도 True로 남아있다.
- 하지만 STOP을 누르면 Isaac Sim physics articulation view는 invalid 된다.
- 다시 PLAY하면 PhysX view는 새로 만들어져야 하는데, 기존 callback이 이미 init된 줄 알고 `get_joint_positions()`를 호출한다.

정석 재실행:

```text
STOP
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_mobile.py').read())
PLAY
```

`pick_place_husky_mobile.py`에는 이 상황을 감지하는 guard를 넣었다:

```text
[M] physics view is not ready. If you pressed STOP, re-exec the script before PLAY for a clean restart.
```

그래도 가장 안정적인 방식은 STOP 후 항상 script를 다시 exec하는 것이다.

## 14. Cube를 직접 옮기고 싶을 때

현재 cube는 `DynamicCuboid`라 PLAY 중에는 PhysX가 위치를 관리한다. 그래서 마우스로 직접 드래그하려 하면 잘 안 되거나 원위치/충돌 반응이 생긴다.

수동 배치 절차:

1. STOP.
2. 스크립트 exec로 cube/stand 생성.
3. 아직 PLAY 누르지 말고 Stage에서:
   - `/MobilePickStand`
   - `/MobilePickCube`
4. 둘을 같이 원하는 위치로 이동.
5. PLAY.

주의:

- 옮긴 뒤 다시 `exec(...)`하면 스크립트가 cube/stand를 다시 원래 위치에 spawn한다.
- 따라서 수동으로 옮긴 뒤에는 바로 PLAY해야 한다.

## 15. 지금까지 만난 주요 문제와 해결

### 15.1 Shape mismatch `(1,7)` vs `(1,9)`

증상:

```text
ValueError: shape mismatch: value array of shape (1,7) could not be broadcast to indexing result of shape (1,9)
```

원인:

- RMPflow는 arm 7개 joint target만 반환.
- 처음에 gripper 2개까지 포함한 9개 index에 적용하려 했다.

해결:

- arm joint 7개에만 `joint_indices` 적용.
- gripper는 `ParallelGripper`가 별도 action으로 처리.

### 15.2 Husky/floor 높이 문제

진단 결과:

```text
/SimpleRoom/Floor/SM_Template_Map_Floor z=-0.815
/husky z=-0.373
```

해결:

- `lift_floor_to_zero.py`로 `/husky`, `/SimpleRoom`을 +0.815m lift.
- floor를 z=0으로 맞춤.
- 실제 walking surface는 z=0.0458.

### 15.3 Cube가 floor에 박힘

원인:

- cube center를 z=0.025에 spawn.
- 하지만 실제 보이는 floor surface가 z=0.0458라 cube가 박혀 보임.

해결:

- `FLOOR_PRIM`에서 surface z를 읽고 `cube_z = floor_z + ...`로 계산.

### 15.4 Franka가 바닥 cube에 못 닿음

원인:

- `panda_link0`가 높은 위치에 있어서 floor 위 cube는 arm reach 아래쪽으로 너무 멀었다.

해결:

- stand를 추가해서 cube를 z 약 0.42m로 올림.

### 15.5 RMPflow가 너무 느림

증상:

- EE가 phase 시간 동안 cube 근처까지 못 감.
- cube_z가 끝까지 고정.

해결:

- PickPlaceController 상태머신은 유지.
- cspace controller를 RMPflow에서 Lula IK direct로 교체.

### 15.6 Lula IK 100% fail

원인:

- Lula IK는 world frame target을 기대.
- 처음에 RMPflow처럼 `panda_link0` local target을 넣었다.

해결:

- `cube.get_world_pose()[0]`를 그대로 picking_position으로 전달.
- top_plate local place는 매 step world로 변환해서 전달.

### 15.7 Gripper가 cube를 제대로 못 잡음

원인:

- IK frame은 `panda_hand`.
- 실제 grasp center는 `panda_hand/tool_center`, 즉 `panda_hand` local +z 0.103m.

해결:

- `end_effector_offset=np.array([0, 0, 0.103])`.
- closed finger position `[0.022, 0.022]`.

### 15.8 Property widget warning

경고:

```text
GeometrySchemaAttributesWidget.build_items took ...
```

의미:

- Isaac Sim UI Property panel이 어떤 prim 속성 목록을 그리는 데 오래 걸렸다는 warning.
- pick/place나 physics 문제 아님.
- 무시해도 된다.

## 16. 다음 단계: lidar로 cube 위치 추정

사용자가 다음으로 하고 싶은 것:

```text
Ground-truth cube pose 대신 lidar PointCloud2 데이터로 cube의 상대 위치를 추정하고 grasp하기.
```

현재 코드에서 교체해야 하는 자리:

`pick_place_husky_mobile.py` 또는 `pick_place_husky.py` 안에서 현재는:

```python
cube_w = np.asarray(cube.get_world_pose()[0], dtype=np.float64)
```

이 부분이 ground-truth이다.

다음 단계는 이 값을:

```python
cube_w = estimate_cube_world_from_lidar()
```

같은 perception 결과로 바꾸는 것이다.

권장 단계:

1. ROS2 `/point_cloud` 또는 현재 Isaac Sim lidar topic 확인.
2. PointCloud2 subscriber 작성.
3. point cloud를 numpy xyz로 변환.
4. ground/floor 제거.
5. ROI 제한:
   - Husky 전방
   - arm reachable distance
   - stand/cube 높이 범위
6. clustering:
   - DBSCAN 또는 간단한 Euclidean clustering
7. cube cluster centroid 계산.
8. lidar frame -> world frame 변환.
   - 현재 RViz Fixed Frame은 `world`.
   - ROS tf가 제대로 publish되고 있는지 확인 필요.
9. `cube_w` 대신 centroid world 좌표를 넣어 PickPlaceController로 전달.

처음에는 완전한 object detection보다 다음처럼 단순화하는 것이 좋다:

- stand 위 파란 cube만 있다고 가정.
- ROI를 stand/cube 예상 위치 주변으로 제한.
- 가장 가까운 cluster 또는 가장 높은 밀도 cluster centroid를 cube로 사용.

중요:

- 현재 mobile pick/place는 cube가 reach band에 들어오면 시작한다.
- lidar 버전도 먼저 "estimated cube xy가 reach band에 들어오면 START" 구조로 가면 된다.
- pick 중에는 추정값을 계속 갱신할지, phase 0~1까지만 갱신하고 이후 latch할지 결정해야 한다.
- 원본 PickPlaceController도 phase 0~1에서 pick target을 저장하고 이후에는 그 값을 쓴다.

추천 구현 파일:

```text
pick_place_husky_lidar.py
```

기존 `pick_place_husky_mobile.py`를 복사해서 시작하는 것이 가장 좋다.

현재 2026-05-29에 1차 lidar perception 연결 파일을 추가했다:

- `lidar_cube_detector.py`
  - `/point_cloud`는 `sensor_msgs/PointCloud2`, fields는 `x,y,z` float32.
  - `/point_cloud` header frame은 `sensor`.
  - `/tf`에 `world -> sensor` transform이 들어온다.
  - detector는 point cloud를 world frame으로 변환하고, z/forward/lateral ROI와 grid clustering으로 cube/stand 후보 cluster를 고른다.
  - `/detected_cube` (`geometry_msgs/PointStamped`)를 publish한다.
  - 동시에 `/tmp/lidar_cube_latest.json`에 최신 target을 쓴다.
  - JSON 예:

```json
{
  "stamp": 1780026357.87,
  "frame_id": "world",
  "x": -0.006,
  "y": 3.440,
  "z": 0.440,
  "points": 158,
  "size_x": 0.249,
  "size_y": 0.113,
  "size_z": 0.107
}
```

- `pick_place_husky_lidar.py`
  - Isaac Script Editor에서 실행.
  - `cube.get_world_pose()` 대신 `/tmp/lidar_cube_latest.json`을 읽어 pick target으로 사용한다.
  - ground-truth cube도 계속 spawn하고 읽어서 `gt_err`를 console에 출력한다. 이건 디버깅용이며 실제 target은 lidar JSON이다.
  - detector가 stale/missing이면 `[L] waiting for lidar estimate ...`를 출력하고 grasp를 시작하지 않는다.

실행 순서:

```bash
cd "/home/user/Desktop/260527 KMS"
./run_lidar_detector.sh
```

주의: 사용자의 기본 셸은 zsh라서 터미널에 직접 `source /opt/ros/jazzy/setup.bash`를 치면 실패할 수 있다. 직접 source하려면 `source /opt/ros/jazzy/setup.zsh`를 쓰거나, 위 wrapper를 사용한다.

Isaac Sim Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_lidar.py').read())
```

그 후 PLAY하고 조이스틱으로 Husky를 천천히 접근시킨다.

### 2026-05-29 종료 시점 perception 상태와 고민

Lidar-only perception은 **시작은 했지만 아직 완성 아님**. 현재 `lidar_cube_detector.py`는 `/point_cloud`에서 후보 cluster를 잡고 `/tmp/lidar_cube_latest.json`까지 쓰는 데 성공했다. 다만 실제 로그를 보면 원하는 cube/stand 후보와 로봇 자체 표면(self-return) 또는 주변 얇은 cluster가 섞여 잡히는 문제가 있었다.

관찰된 좋은 후보 예:

```text
[lidar] raw=4879 cand=171 clusters=2
centroid=(+0.007,+3.445,+0.375)
target=(+0.007,+3.445,+0.431)
sensor=(+1.027,-0.034,-0.263)
size=(0.242,0.133,0.090) n=143 score=0.428
```

해석:

- `sensor x≈1.0m`, `sensor y≈0` 근처.
- cluster 크기 `size≈0.24 x 0.13 x 0.09`, 점 수 `n≈100+`.
- stand/cube가 같이 잡힌 큰 후보로 보이며, target z를 cluster max z로 잡아 약 `0.43m`가 나왔다.

관찰된 나쁜 후보 예:

```text
[lidar] raw=2392 cand=19 clusters=1
centroid=(-0.243,+2.842,+0.440)
target=(-0.243,+2.842,+0.441)
sensor=(+0.432,+0.234,-0.200)
size=(0.047,0.027,0.002) n=19 score=0.467
```

해석:

- `sensor x≈0.43m`, `sensor y≈±0.24m`로 거의 고정.
- `size_z≈0.001~0.002`라 매우 얇은 cluster.
- Husky/lidar mount/robot body 일부가 point cloud에 잡힌 self-return일 가능성이 큼.

이에 따라 `lidar_cube_detector.py`에 1차 보정 추가:

- 가까운 자기 표면 필터:

```python
SELF_FILTER_FORWARD = 0.55
SELF_FILTER_LATERAL_ABS = 0.32
```

- 너무 얇고 작은 cluster 제거:

```python
MIN_CLUSTER_Z_SIZE = 0.025
MIN_CLUSTER_XY_SIZE = 0.08
```

하지만 여기서 마무리했기 때문에, 이 필터가 실제로 충분한지 아직 최종 검증하지 않았다.

현재 고민 지점:

1. **Lidar-only로 원하는 물체를 안정적으로 perception할 수 있을까?**
   - 가능은 해 보이지만 ROI/cluster/filter 튜닝이 필요하다.
   - 특히 stand, cube, robot self-return, floor/wall fragment가 point cloud에서 섞인다.
   - 단순 centroid는 stand+cube 전체 중심으로 내려갈 수 있어서 target z는 `max_z`를 쓰도록 바꿨다.
   - x/y도 stand+cube 전체 centroid라 실제 cube 중심과 약간 다를 수 있다.

2. **RGB camera vision을 병렬로 써야 할까?**
   - 파란 cube처럼 색/형태가 명확한 물체라면 RGB segmentation이 물체 식별에는 훨씬 강할 수 있다.
   - Lidar는 3D 거리/좌표, RGB는 object identity/segmentation에 강점이 있다.
   - 현실적인 다음 방향은 RGB로 "어떤 cluster가 cube인지"를 고르고, lidar/depth/point cloud로 3D 위치를 얻는 sensor fusion일 수 있다.
   - 다만 당장 최소 구현은 lidar-only ROI + clustering으로 계속 튜닝하는 것이 빠르다.

추천 다음 실험:

1. `./run_lidar_detector.sh`를 켜고 Husky를 천천히 움직이면서 좋은 후보가 계속 선택되는지 확인.
2. `size`, `n`, `sensor=(x,y,z)`, `target=(x,y,z)` 로그를 보고 self-return이 제거됐는지 확인.
3. `pick_place_husky_lidar.py`를 실행해 `[L] WAIT ... gt_err=...`가 얼마나 나오는지 확인.
4. `gt_err`가 5~10cm 이상 흔들리면 바로 grasp하지 말고 detector 튜닝.
5. Lidar-only가 불안정하면 RGB camera 추가를 검토:
   - Isaac Sim에 RGB camera sensor 추가 또는 기존 camera topic 확인.
   - RGB에서 blue cube segmentation.
   - segmentation mask에 해당하는 point cloud/depth만 추출.
   - 그 centroid를 pick target으로 사용.

## 17. 발표 자료로 쓸 수 있는 핵심 문장

### 예제 분석

```text
Franka pick-and-place 예제는 시나리오는 10-phase 상태머신으로 고정되어 있고,
pick 위치는 cube USD prim의 ground-truth pose를 매 step 읽으며,
관절 명령은 RMPflow motion policy가 매 step 생성한다.
```

### 결합 로봇 이식

```text
기존 예제는 Franka가 독립 articulation이고 base가 고정되어 있다는 가정 위에 있다.
Husky+Franka 결합 로봇에서는 /husky가 단일 articulation root이므로,
SingleArticulation('/husky')를 사용하고 joint_indices로 arm 7개 관절만 제어해야 한다.
```

### RMPflow vs Lula IK

```text
RMPflow는 부드러운 motion policy지만 결합 로봇 시나리오에서는 응답이 너무 느려
phase 시간 안에 cube에 도달하지 못했다. 따라서 상태머신은 유지하고,
motion generation 부분만 Lula IK solver 기반 direct IK 방식으로 교체했다.
```

### Ground-truth to lidar

```text
현재 pick target은 cube.get_world_pose()로 얻은 simulation ground-truth이다.
최종 목표는 이 좌표 출처를 lidar PointCloud2 기반 object centroid 추정값으로 교체하는 것이다.
```

### Perception 고민

```text
Lidar-only perception은 3D 위치를 직접 얻을 수 있다는 장점이 있지만,
robot self-return과 stand/cube cluster ambiguity가 있어 ROI와 clustering tuning이 필요하다.
RGB camera를 병렬로 쓰면 object identity/segmentation을 보완할 수 있으므로,
향후에는 RGB로 cube를 식별하고 lidar/point cloud로 3D 위치를 얻는 sensor fusion도 고려 중이다.
```

## 18. 새 AI가 바로 이어서 할 일

추천 순서:

1. `husky_franka.usd` 열고 `pick_place_husky.py` 실행해서 정지 상태 성공 재확인.
2. `run_controller.sh`로 joystick controller 실행.
3. `pick_place_husky_mobile.py` 실행해서 움직이는 Husky 접근 후 pick/place 테스트.
4. `./run_lidar_detector.sh` 실행해서 lidar cluster 후보가 안정적으로 잡히는지 확인.
5. `pick_place_husky_lidar.py` 실행해서 `[L] WAIT ... gt_err=...` 로그 확인.
6. `gt_err`, `target`, `sensor`, `size`, `n`을 보며 lidar-only detector 튜닝.
7. self-return/stand ambiguity가 계속 크면 RGB camera 병렬 활용 검토.
8. 오차가 충분히 작고 안정적이면 실제 grasp target으로 사용한다.

## 19. 빠른 실행 레시피

### 컨트롤러 켜기

```bash
cd "/home/user/Desktop/260527 KMS"
DISPLAY=:0.0 ./run_controller.sh
```

### 정지 Husky pick/place

Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky.py').read())
```

### RMPflow 실패 비교

Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_rmp.py').read())
```

### 움직이는 Husky pick/place

Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_mobile.py').read())
```

### Lidar detector

Terminal:

```bash
cd "/home/user/Desktop/260527 KMS"
./run_lidar_detector.sh
```

### Lidar 기반 pick/place 시도

Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_lidar.py').read())
```

## 20. Husky Lidar SLAM 전환 시작

2026-05-29 후반에 Manipulator/perception은 잠시 멈추고, Husky Lidar 기반 SLAM을 다루는 방향으로 전환하기로 했다.

전환 이유:

- Manipulator는 ground-truth pick/place까지 성공했고, lidar-only object perception은 self-return/cluster ambiguity 때문에 추가 perception tuning이 필요하다.
- SLAM은 모바일 로봇 파트의 핵심이며, 최종 스토리상 "Husky가 map/pose를 얻고 물체 근처로 이동한 뒤 Franka가 grasp"로 자연스럽게 이어진다.

현재 ROS topic 상태:

```text
/cmd_vel      [geometry_msgs/msg/Twist]
/point_cloud  [sensor_msgs/msg/PointCloud2]
/tf           [tf2_msgs/msg/TFMessage]
```

처음 확인 시 `/scan`, `/odom`, `/map`은 없었다.

중요한 구조:

- Isaac Sim lidar는 `/point_cloud`를 publish한다.
- `slam_toolbox`는 일반적으로 `/scan` (`sensor_msgs/LaserScan`)을 기대한다.
- 따라서 `PointCloud2 -> LaserScan -> slam_toolbox` 경로로 시작한다.

설치한 ROS 패키지:

```bash
sudo apt-get install -y ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan
```

추가된 파일:

```text
run_pointcloud_to_scan.sh
run_slam_toolbox_husky.sh
slam_toolbox_husky_params.yaml
```

`run_pointcloud_to_scan.sh`:

- `/point_cloud`를 `/scan`으로 변환한다.
- `target_frame=sensor`.
- 높이 필터는 첫 실험값으로 `min_height=-0.40`, `max_height=0.30`.
- 이 값은 실제 `/scan` 품질을 보며 조정 필요.

`slam_toolbox_husky_params.yaml`:

- 첫 실험에서는 Isaac이 `/tf`로 `world -> sensor`를 주는 것을 활용한다.
- `odom_frame=world`
- `base_frame=sensor`
- `map_frame=map`

주의:

- 이것은 정식 모바일 로봇 TF 구조는 아니다.
- 정식 구조는 보통 `map -> odom -> base_link -> sensor`.
- 현재 첫 실험은 wheel odometry 없이 `world`를 odom처럼 쓰고 lidar frame을 base처럼 쓰는 빠른 검증용이다.
- 나중에 `/odom` 또는 Isaac의 odometry publisher를 추가하면 `odom_frame=odom`, `base_frame=base_link`로 바꾸는 것이 맞다.

실행 순서 후보:

Terminal 1:

```bash
cd "/home/user/Desktop/260527 KMS"
./run_pointcloud_to_scan.sh
```

Terminal 2:

```bash
cd "/home/user/Desktop/260527 KMS"
./run_slam_toolbox_husky.sh
```

Terminal 3:

```bash
cd "/home/user/Desktop/260527 KMS"
DISPLAY=:0.0 ./run_controller.sh
```

Isaac Sim:

- `husky_franka.usd` 또는 lidar가 있는 Husky 씬 열기.
- PLAY.
- 조이스틱으로 천천히 주행.

검증 명령:

```bash
source /opt/ros/jazzy/setup.zsh
ros2 topic list -t
ros2 topic echo /scan --once --no-daemon
ros2 topic echo /map --once --no-daemon
```

기대:

- `/scan [sensor_msgs/msg/LaserScan]` 생성.
- slam_toolbox가 `/map [nav_msgs/msg/OccupancyGrid]` publish.
- RViz에서 Fixed Frame을 `map`으로 하고 `Map`, `LaserScan`, TF를 확인.

다음 AI가 이어서 할 일:

1. Isaac Sim PLAY 상태에서 `run_pointcloud_to_scan.sh` 실행 후 `/scan`이 실제로 나오는지 확인.
2. `/scan`이 비어 있거나 이상하면 `min_height/max_height/range_min/range_max` 조정.
3. `/scan`이 안정적이면 `run_slam_toolbox_husky.sh` 실행.
4. `/map` topic이 생기는지 확인.
5. RViz config를 SLAM용으로 새로 저장.
6. TF가 문제면 임시 설정(`odom_frame=world`, `base_frame=sensor`) 대신 정식 `odom/base_link/sensor` 구조를 구성.

### Pose 진단

Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/diagnose_poses.py').read())
```

### Floor 진단

Script Editor:

```python
exec(open('/home/user/Desktop/260527 KMS/diagnose_floor.py').read())
```
