# Practice 2

## 1. 실습 목표
- ROS2 노드 구조와 토픽 통신 방식 이해
- `f110-GymROS`를 활용한 Wall Following 알고리즘 구현 및 검증

---
## 2. 학습 내용
- **작성자** : 남지연
- **작성일** : 2025-09-28 (일)
- **목표:** <br>
RViz와 f1tenth_gym_ros 시뮬레이터를 활용해 `/scan` 데이터를 확인하고, `/drive` 토픽으로 직접 조향각과 속도를 퍼블리시하여 차량의 움직임을 실시간으로 관찰한다. 이를 통해 LiDAR 데이터의 변화를 이해하고 조향각 변화가 차량 주행에 미치는 영향을 실습한다.<br>

**<학습 내용 정리> - (1)**<br>
---
: 본 실습에서는 `wall_follower_node` 대신 직접 터미널에서 `/drive` 토픽을 퍼블리시하여 조향각 및 속도 제어 실험을 진행하였다.<br>

**1) 환경 준비**
  - sim_ws 워크스페이스를 빌드하고 source install/setup.bash로 ROS 2 환경을 설정하였다.
  - `ros2 launch f1tenth_gym_ros gym_bridge_launch.py` 명령으로 시뮬레이터와 RViz를 실행하였다.
  - `ros2 topic echo /scan`을 통해 LiDAR 센서 값이 시뮬레이터에서 정상적으로 퍼블리시되고 있음을 확인하였다.
  - `ros2 topic info /drive`로 차량 제어 명령을 받을 수 있는 구독자가 존재함(브리지 노드)을 확인하였다.
<br>

**2) 차량 제어 실험**
  - 별도의 제어 노드를 만들지 않고, 새로운 터미널에서 ROS 2 환경을 소스한 후 `/drive` 토픽으로 AckermannDriveStamped 메시지를 직접 퍼블리시하였다.
  - speed는 1.0으로 고정하고, steering_angle을 0.0, 0.3, -0.3 등으로 수동 변경하며 차량이 직진, 좌회전, 우회전하는 반응을 실험하였다
<br>

**3) 결과 및 관찰**
  - 조향각을 변경하면 RViz에서 차량 모델이 회전하고, 그에 따라 `/scan` 토픽의 거리값 배열이 변하는 것을 확인하였다.
  - `steering_angle`이 양수일 때 왼쪽으로, 음수일 때 오른쪽으로 차량이 회전함을 실습으로 검증하였다.
  - 센서 값은 단순히 확인만 하고, 제어 로직에는 사용하지 않았으므로 차량은 항상 지정된 `steering_angle`로만 움직였다.
<br>

**4) 검증**
  - `rqt_graph`에서 `/drive` 토픽 구독자(/bridge 노드)가 존재함을 확인하여 제어 명령이 시뮬레이터에 전달될 준비가 되어 있음을 검증하였다.
  - `/scan` 토픽은 퍼블리시되고 있으나 서브스크라이버가 없어 `/scan → (노드)` 연결선은 나타나지 않는다.

  - 실행하지 않고 있을 때<br><br>
    <img width="366" height="400" alt="image" src="https://github.com/user-attachments/assets/caab89c1-be99-44fa-b77c-050a8e2d3fde" />

    <br>


  - 실행하였을 때<br>
  : 브리지가 `/drive` 구독 중이므로 차량이 움직일 준비 완료<br>
    <img width="440" height="415" alt="image" src="https://github.com/user-attachments/assets/5fec54fc-7ddb-40dc-8d44-cacfb77254c7" />
    <br>
    <img width="2045" height="606" alt="image" src="https://github.com/user-attachments/assets/4908a471-a607-4ce0-a840-e38e5df94fc6" />


**<학습 내용 정리> - (2)**<br>
---
본 실습에서는 `wall_follower_node`를 통해 `/drive` 토픽을 퍼블리시하여 조향각 및 속도 제어 실험을 진행하였다.<br>

<img width="2078" height="542" alt="image" src="https://github.com/user-attachments/assets/9c86f76d-83b7-40e2-93fa-226831c39571" />
<br>

  - 첫 번째 터미널
```bash
cd ~/sim_ws
. install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

- 두 번째 터미널
```bash
cd ~/sim_ws
. install/setup.bash
ros2 run wall_following_pkg wall_follower_node
```

- 세 번째 터미널
```bash
cd ~/sim_ws
. install/setup.bash
rqt_graph
```


## 3. 실습 결과
---
### 1. 윤우린 (09/29)

#### 이론 학습
- PID 제어
- Pure Pursuit
- SLAM
  
#### wall_follow_node.py 분석
- `WallFollow` 클래스는 `rclpy.node.Node`를 상속 → ROS2 노드.
- `/scan` 토픽 구독 (라이다 센서값: `LaserScan`)
- `/drive` 토픽 발행 (차량 제어: `AckermannDriveStamped`)
  
#### wall_follow_node.py 핵심 method
1. get_range(range_data, angle) 
: 특정 각도에서 라이다 거리값을 리턴하는 함수
2. get_error(range_data, dist) 
: 벽과의 목표거리(dist)와 실제 측정 거리의 차이를 계산
3. pid_control(error, velocity) 
: PID제어기로 조향각 결정 & AckermannDriveStamped 메시지 채워서 /drive 토픽 발행
4. scan_callback(msg)
: /scan 들어올 때마다 실행됨 & get_error() => pid_control() 호출해서 실제 차량/시뮬레이터를 제어함

#### 코드 실행 흐름
- ros2 run or ros2 launch로 노드 실행
- /scan구독 => scan_callback 실행 => 에러 계산 => PID로 조향 명령 발행
  
#### 실습 결과 화면 
<img width="900" height="384" alt="Image" src="https://github.com/user-attachments/assets/fcfcc659-d25f-49cf-9a03-eb401e578060" />

#### 터미널 로그 
<img width="553" height="178" alt="Image" src="https://github.com/user-attachments/assets/1c7450eb-eb32-497f-a956-01e7d35a5717" />

#### 문제 발생 및 해결
1. 문제 : ros2 run wall_follow wall_follow 실행 후에 시뮬레이터에서 차량이 움직이지 않음
2. 해결 흐름
   - 토픽 연결 확인 : ros2 topic list
   - dirve 메세지 정보 확인 : ros2 topic info /drive
   => publisher count가 0인 문제 발생. 
   ##### 즉, wall_follow 노드가 /drive에 메시지 발행을 안하는 상황임.
---
### 2. 남지연 (09/28)
<br>
##### 첫 번째 실습
<img width="780" height="317" alt="image" src="https://github.com/user-attachments/assets/cdad5491-2740-4d3f-875a-87e3c790464d" />

<br> 
게속해서 퍼블리시하는 모습  <br>
<img width="780" height="317" alt="image" src="https://github.com/user-attachments/assets/3a0115e0-9328-40eb-bb46-cc76501db94c" />

##### 두 번째 실습
<img width="440" height="415" alt="image" src="https://github.com/user-attachments/assets/c1ca8f7b-b71f-47fa-90b3-327a3d42c9a7" />
