# Practice 2: gym_ros 실습 
<img width="600" height="350" alt="image" src="https://github.com/user-attachments/assets/2cd42a1c-9269-413e-aae7-df45d9c448e9" />


## 1. 실습 목적
- ROS2 퍼블리셔–서브스크라이버 구조와 토픽 통신 이해
- LiDAR 센서 데이터를 구독하고 제어 명령을 퍼블리시하는 노드 구현
- f110-GymROS 시뮬레이터를 활용한 Wall Following 알고리즘 실습
- rqt 도구를 통한 토픽 연결 및 데이터 변화 확인<br><br><br>

## 2. 실습 과제
**1) 환경 준비**
  - `sim_ws` 워크스페이스 준비
  - `sim_ws/src` 안에 새 패키지 생성 (예: wall_following_pkg)

**2) 노드 구현**
  - 구독: `/scan` (LiDAR 데이터)
  - 퍼블리시: `/drive` (speed=1, steering=조건에 맞게 변경)
  - 조건:
    - 거리가 ≤ 1 → `steering_angle` = 0.5
    - 기본 speed는 항상 1
      
**3) 실험**
  - steering 값을 바꿔가면서 차량 반응 관찰 (튕겨나감, 붙어서 달림 등)

**4) 검증**
  - rqt_graph: `/scan` → `wall_follower_node` → `/drive` 연결 확인
  - rqt_plot: 라이다 값 변화 확인 (선택적)<br><br><br>

## 3. 제출 방식
- `practice.md` 파일에 실습 과정 기록
- `practice.md` 파일에 각자 결과 이미지 첨부
- 제출 기간 : ~ 9/29 (월)
