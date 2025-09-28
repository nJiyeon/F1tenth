# Practice 2

## 1. 실습 목표
- ROS2 노드 구조와 토픽 통신 방식 이해
- `f110-GymROS`를 활용한 Wall Following 알고리즘 구현 및 검증

---
- **작성자** : 남지연
- **작성일** : 2025-09-28 (일)
- **목표:** <br>
RViz와 f1tenth_gym_ros 시뮬레이터를 활용해 `/scan` 데이터를 확인하고, `/drive` 토픽으로 직접 조향각과 속도를 퍼블리시하여 차량의 움직임을 실시간으로 관찰한다. 이를 통해 LiDAR 데이터의 변화를 이해하고 조향각 변화가 차량 주행에 미치는 영향을 실습한다.<br>

**<학습 내용 정리>**<br>

**1)환경 준비**
  - `sim_ws` 워크스페이스를 빌드하고 `source install/setup.bash`로 환경 설정을 완료했다.
  - `ros2 launch f1tenth_gym_ros gym_bridge_launch.py` 명령으로 시뮬레이터 및 RViz를 실행하고 `/scan` 토픽이 정상적으로 수신되는지 확인하였다.
  - `ros2 topic echo /scan`을 통해 LiDAR 거리값이 실시간으로 출력되는 것을 확인하였다.
<br>

**2) 차량 제어 실험**
  - 새로운 터미널에서 ROS 2 환경을 소스한 뒤 `/drive` 토픽으로 AckermannDriveStamped 메시지를 직접 퍼블리시하였다.
 - speed는 1.0으로 고정하고, steering_angle을 0.0, 0.3, -0.3 등으로 변경하여 차량이 직진, 좌회전, 우회전하는 반응을 관찰하였다.
<br>

**3) 결과 및 관찰**
  - 조향각을 변경하자 RViz에서 차량 모델이 회전하며 벽과의 거리가 달라지고, `/scan` 값이 변하는 것을 확인할 수 있었다.
  - `steering_angle`이 양수일 때 차량은 왼쪽으로, 음수일 때 오른쪽으로 회전하는 것을 실습으로 검증하였다.
<br>

**4) 검증**
  - `rqt_graph`에서 `/drive` 토픽 퍼블리시 노드가 활성화된 것을 확인하였다.

  - 실행하지 않고 있을 때<br><br>
    <img width="366" height="400" alt="image" src="https://github.com/user-attachments/assets/caab89c1-be99-44fa-b77c-050a8e2d3fde" />

    <br>


  - 실행하였을 때<br>
  : 브리지가 `/drive` 구독 중이므로 차량이 움직일 준비 완료<br>
    <img width="440" height="415" alt="image" src="https://github.com/user-attachments/assets/5fec54fc-7ddb-40dc-8d44-cacfb77254c7" />
    <br>
    <img width="2045" height="606" alt="image" src="https://github.com/user-attachments/assets/4908a471-a607-4ce0-a840-e38e5df94fc6" />


---
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
<img width="780" height="317" alt="image" src="https://github.com/user-attachments/assets/cdad5491-2740-4d3f-875a-87e3c790464d" />

<br> 
게속해서 퍼블리시하는 모습  <br>
<img width="780" height="317" alt="image" src="https://github.com/user-attachments/assets/3a0115e0-9328-40eb-bb46-cc76501db94c" />
