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
