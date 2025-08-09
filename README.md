# F1TENTH ROS2 (Foxy) 시뮬 + 텔레옵 + PID(Ackermann 변환) 빠른 가이드
환경

    Ubuntu 20.04, ROS2 Foxy

    (옵션) NVIDIA GPU 없음

    워크스페이스: ~/sim_ws

## 1) F1TENTH Gym 설치 (Python)
```
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
```
## 2) ROS2 워크스페이스 & 시뮬 패키지
```
cd ~
mkdir -p sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros
```
(선택) 맵 파일 경로

    맵은 다음 경로 사용: <home>/sim_ws/src/f1tenth_gym_ros/maps/levine

## 3) 의존 패키지 설치 (rosdep)
```
source /opt/ros/foxy/setup.bash
cd ~/sim_ws
rosdep install -i --from-path src --rosdistro foxy -y
```
## 4) 빌드 & 환경 설정
```
cd ~/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
## 5) 시뮬레이터 실행
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
## 6) 텔레옵 (키보드)

필요 패키지:
```
sudo apt install -y ros-foxy-teleop-twist-keyboard
```
실행:
```
cd ~/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
키:

Moving around:
   u    i    o
   j    k    l
   m    ,    .

## 7) RViz를 map 기준으로 보기(로컬라이제이션 없이)

정적 TF(시각화용)
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
 [주의] AMCL/SLAM 켤 땐 이 정적 TF를 끄세요(중복 TF 충돌 방지).

## 8) PID 속도 컨트롤러 + Ackermann 변환 노드
의존 패키지
```
sudo apt install -y ros-foxy-ackermann-msgs
```
패키지 구조
```
~/sim_ws/src/twist_to_ackermann_pid/
├─ package.xml
├─ setup.py
├─ resource/
│   └─ twist_to_ackermann_pid        # 파일 내용에 'twist_to_ackermann_pid' 한 줄
└─ twist_to_ackermann_pid/
    ├─ __init__.py
    └─ twist_to_ackermann_pid_node.py
```
노드 요약
    입력: /cmd_vel (geometry_msgs/Twist)
    상태: /odom (nav_msgs/Odometry) ← 실행 시 /ego_racecar/odom으로 리맵
    출력: /drive (ackermann_msgs/AckermannDriveStamped)
    기능: 속도 PID + 자전거 모델로 Ackermann 조향각 변환
    δ=atan(L⋅ω/max⁡(∣v∣,0.2))δ=atan(L⋅ω/max(∣v∣,0.2))
빌드 & 실행
```
cd ~/sim_ws
colcon build --symlink-install
source install/setup.bash
```
### 예시 파라미터 + 오돔 리맵
```
ros2 run twist_to_ackermann_pid node --ros-args -r /odom:=/ego_racecar/odom \
  -p kp:=1.0 -p ki:=0.3 -p kd:=0.06 -p max_speed:=3.0
```
주행 중 즉석 튜닝(다른 터미널)
```
ros2 param set /twist_to_ackermann_pid kp 1.2
ros2 param set /twist_to_ackermann_pid ki 0.40
ros2 param set /twist_to_ackermann_pid kd 0.08
ros2 param set /twist_to_ackermann_pid max_speed 3.5
ros2 param set /twist_to_ackermann_pid max_steering_deg 28.0
```
    Ackermann 관련 핵심 파라미터: wheelbase, max_steering_deg
    max_speed는 PID 출력 상한(간접적으로 조향 계산에도 영향)

9) 빠른 점검 체크리스트

 노드 존재
ros2 node list | grep twist_to_ackermann_pid
# /drive 연결 상태 (Publisher 1, Subscriber 1)
ros2 topic info /drive
# 퍼블리시 속도(≈50 Hz 권장)
ros2 topic hz /drive
# (옵션) RViz: Fixed Frame = map (정적 TF 켠 상태) 또는 odom

10) 트러블슈팅
    /drive에 메시지 안 나옴
        퍼블리셔 0 → PID 노드 미실행/크래시 → 노드 재실행, 터미널에 로그 확인
        텔레옵이 /cmd_vel을 내보내는지 확인: ros2 topic echo /cmd_vel

    ModuleNotFoundError (패키지 못 찾음)
        __init__.py, resource/… 파일, setup.py의 console_scripts 확인
        빌드 후 각 터미널마다 source install/setup.bash

    ros2 topic echo /drive | head -n 5 뒤 에러
        BrokenPipeError는 정상(파이프 닫혀서 발생). 대신 -n 옵션 사용:
        ros2 topic echo -n 5 /drive

    RViz에서 map 기준으로 보고 싶은데 안 보임
        정적 TF 실행(위 7절) 또는 Fixed Frame을 odom으로 전환
        AMCL/SLAM 켰으면 정적 TF는 끄기

11) 참고 토픽
    명령 속도: /cmd_vel
    실제 속도(오돔): /ego_racecar/odom
    구동 명령: /drive

12) 텔레옵 팁
    k 즉시 정지, i 가속, , 감속, j/l 좌/우
    위험하면 max_speed를 낮춰서 튜닝한 뒤 점진적으로 올리기
