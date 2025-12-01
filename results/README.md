## F1tenth Autonomous Driving System

ROS2 기반 인지·위치추정·제어 통합 패키지

본 저장소는 F1tenth 자율주행 플랫폼에서 사용되는 LiDAR 기반 인지 모듈, 파티클 필터 위치추정, MPPI(Predictive) 제어기, Pure Pursuit 제어기, waypoint 로깅 유틸리티, 전체 시스템 런치 파일을 포함한 통합 프로젝트이다.

일부 MPPI 구현 코드는 저작권 문제로 원본을 repository에서 제거하였으며, 사용자 측에서 직접 구현하거나 외부 오픈소스 대체 모듈을 연결해야 한다.

### 1. Repository 구조

```text
results/
 ├── f1tenth_mppi/           # MPPI 경로계획·제어 패키지 (일부 코드 제거됨)
 ├── f1tenth_system/         # 전체 시스템 런치파일, 통합 실행 환경
 ├── particle_filter/        # LiDAR 기반 Particle Filter localization
 ├── pure_pursuit/           # Pure Pursuit 기반 조향 제어기
 ├── ros2_py_racecar/        # racecar 인터페이스 및 공용 메시지/노드
 ├── waypoint_logger/        # 웨이포인트 기록 유틸리티
 └── README.md
```

### 2. 요구 환경
| 항목 | 버전 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble |
| Python | 3.10+ |

추가로 RViz2, Gym-ROS 등 시각화·시뮬레이션 툴을 사용할 수 있다.

### 3. 설치

#### 3.1 저장소 클론
```bash
git clone https://github.com/<your-id>/f1tenth_system.git
cd f1tenth_system
```

#### 3.2 ROS2 워크스페이스 생성 및 빌드
```bash
mkdir -p ~/f1_ws/src
cp -r results ~/f1_ws/src/
cd ~/f1_ws
colcon build
source install/setup.bash
```

#### 4. 라이선스 및 주의사항
- MPPI 관련 핵심 알고리즘 일부는 저작권 문제로 제거되어 있으며, 사용자가 직접 구현하거나 외부 라이브러리를 연동해야 한다.
- ROS2 패키지 또는 기타 third-party 모듈 설치가 추가로 필요할 수 있다.
- 본 코드는 연구 및 교육 목적 사용을 우선으로 한다.
