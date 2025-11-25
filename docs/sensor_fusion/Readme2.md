
# **Sensor Fusion(센서 퓨전) 정리**

---

# 센서 퓨전이란?

여러 종류의 센서 데이터를 결합해 더 정확하고 신뢰성 있는 정보를 얻는 기술

자율주행에서는 센서 하나만 사용할 시 문제점

- 환경 인식 정확도 부족
- 신뢰성 부족
- 특정 상황에서 blind spot 발생

이 있기 때문에 센서들을 결합해 보완하는 것이 필수.

---

# 센서 퓨전이 필요한 이유

| 센서 | 장점 | 단점 |
| --- | --- | --- |
| **LiDAR** | 거리 정확도 높음, 3D 정보 확보 | 비싸고 비·눈·안개에 약함 |
| **Camera** | 객체 인식 강함, 비용 낮음 | 거리 추정 부정확, 어두운 환경 약함 |
| **GPS** | 글로벌 위치 | 터널/도심에서 끊김 |
|  |  |  |
|  |  |  |

⇒개별 센서의 약점을 서로 보완하기 위해 센서 퓨전이 필요함.

---

# 센서 퓨전의 종류

### 1) **Low-Level Fusion (raw-data fusion)**

센서의 **원시(raw)** 데이터를 바로 합치는 방식

예: LiDAR + Radar 포인트 클라우드 병합

장점: 정보 손실 적음

단점: 구현 및 계산 비용 큼

---

### 2) High-Level Fusion (decision fusion)

센서별로 이미 전처리된 **결과값**을 결합

예:

- 카메라: 차량/보행자 detection 결과
- LiDAR: centroid + bounding box
- Radar: velocity

장점: 구현 쉬움

단점: raw 정보 손실 있음

---

### 3) Mid-Level Fusion (feature fusion)

센서에서 추출된 특징을 결합

예: 이미지 + point cloud feature 결합

→ BEV 기반 3D 객체 인식에서 많이 사용됨

---

# 주요 센서 퓨전 알고리즘

## **(1) 칼만 필터(Kalman Filter, KF)**

- 선형 시스템에서 잡음 포함 상태를 추정하는 알고리즘
- IMU + wheel odometry + GPS 융합에 사용

---

## **(2) 확장 칼만 필터(EKF, Extended KF)**

- 비선형 시스템에 KF 적용한 버전
- 자율주행 Localization에서 가장 많이 사용되는 필터
- 예:
    - Wheel odometry + IMU + LiDAR(NDT)
    - GPS + IMU + LiDAR map matching
    - ROS2 → `ekf_localization_node(rviz)`가 대표적

---

## **(3) Particle Filter**

- AMCL과 같이 복수의 particle을 생성하여 위치 추정
- Global localization에 강함

---

# 자율주행에서의 Sensor Fusion 적용 분야

## (1) Localization

다양한 센서 입력을 합쳐 차량의 **정확한 위치·자세(pose)** 를 추정

- GPS + IMU + Wheel odometry + LiDAR(NDT/ICP)
    
    → EKF로 결합
    
- Autoware, Apollo의 Localization 모듈

---

## (2) Perception(객체 인식)

LiDAR + Camera + Radar 결합

- LiDAR: 정확한 거리, 3D bbox
- Camera: 객체 분류/semantic
- Radar: 속도(v_r), 움직임 추적

---

## (3) Tracking(추적)

- Kalman Filter 기반 multi-object tracking
- LiDAR + Radar로 속도 + 위치 추정
- Camera로 semantic 추가

---

## (4) Planning & Control

센서 퓨전으로 추정된 상태값(state)을 기반으로

주행 경로 생성 및 제어 정확도 향상

---

# 센서 퓨전의 흐름 (ROS2 기준)

1. 센서별 topic 수신
    - `/camera/image_raw`
    - `/lidar/points`
    - `/imu/data_raw`
    - `/gps/fix`
2. Sensor Fusion 노드(EKF/UKF) 실행
    - `/ekf/filtered_odom` 생성
3. Localization 결과는 Navigation, Planning. Control에 모두 사용됨

---

# 센서 퓨전의 문제점

- 타이밍 동기화(Sync) 문제
    
    →  Camera + LiDAR 상황일때 심함
    
- 센서 coordinate 변환
    
    → Calibration 필수
    
- 노이즈 모델 설정
    
    → R matrix, Q matrix 튜닝
    
- 센서별 가용성
    
    → GPS 끊김 상황, LiDAR occlusion 등
    

---
