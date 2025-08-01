# f1tenth

## F1TENTH ROS2 Foxy Docker + noVNC
## Ubuntu 20.04 + ROS2 Foxy 환경에서 NVIDIA GPU 없이 Docker + noVNC로 실행하는 F1TENTH 시뮬레이터입니다.

필수 패키지 설치

sudo apt update
sudo apt install \
  docker.io docker-compose \
  ros-foxy-tf2-geometry-msgs \
  ros-foxy-ackermann-msgs \
  ros-foxy-joy \
  ros-foxy-xacro \
  ros-foxy-nav2-map-server \
  ros-foxy-teleop-twist-keyboard \
  ros-foxy-nav2-bringup -y

시뮬레이터 코드 클론 및 빌드

mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
cd ~/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select f1tenth_gym_ros

Docker 실행

docker-compose.yml 작성 예시

version: '3'
services:
  sim:
    build: .
    container_name: f1tenth_sim
    ports:
      - "8080:8080"
    volumes:
      - .:/sim_ws/src/f1tenth_gym_ros
    tty: true

실행
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
cd f1tenth_gym_ros
docker-compose up

브라우저 접속: http://localhost:8080/vnc.html

시뮬레이터 구성 (다중 터미널)

### 터미널 1: 시뮬레이터 실행
source /opt/ros/foxy/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

### 터미널 2: 맵 서버 실행
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=$HOME/f1tenth_ws/src/f1tenth_gym_ros/maps/levine.yaml

### 터미널 3: 맵 활성화
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

### 터미널 4: TF 변환
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map ego_racecar/base_link

### 터미널 5: 키보드 조작
ros2 run teleop_twist_keyboard teleop_twist_keyboard

### 터미널 6: Twist to Ackermann
ros2 run cmdvel_to_ackermann cmdvel_to_ackermann

### 터미널 7: PID 제어
ros2 run pid_velocity_controller pid_velocity_controller

## 키보드 조작
| 키 | 기능 |
|----|------|
| i  | 전진 |
| ,  | 후진 |
| k  | 정지 |
| u/o | 전진+회전 |
| m/. | 후진+회전 |
| q/z | 속도 증가/감소 |


참고 문헌 및 라이선스

J. O'Kelly et al., “F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning,” NeurIPS 2019

본 프로젝트는 MIT License를 따릅니다.

