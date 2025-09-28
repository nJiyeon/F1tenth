# Practice 2

## 1. 실습 목표
- ROS2 노드 구조와 토픽 통신 방식 이해
- `f110-GymROS`를 활용한 Wall Following 알고리즘 구현 및 검증

---
## 2. 학습 내용
  
- **작성자** : 
- **작성일** : 2025-09-29
- **목표:** 

**<학습 내용 정리>**<br>


---
## 3. 실습 결과

### 1. 윤우린 (09/29)

#### 이론 학습
- PID 제어
- Pure Pursuit
- SLAM
  
#### wall_follow_node.py 분석
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

