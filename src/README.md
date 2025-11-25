# f1tenth korea chamionship에서 사용된 mppi와 fgm 알고리즘
# 🚗 개요

## 🔷 FGM(Follow-the-Gap Method)

FGM은 LiDAR 스캔에서 가장 큰 “빈 공간(Gap)”을 찾고,

그 공간을 향해 각도를 조절하여 충돌 없이 주행하는 알고리즘입니다.

FGM 특징:

- 계산량이 적어 고속 실시간 제어 가능
- 근거리 장애물 회피 능력 우수
- ROS2에서 매우 안정적으로 동작

본 구현에서는 다음 기능을 포함합니다:

- LiDAR 스캔 smoothing
- 인접 빔 간 안전 거리 체크
- 최대 Gap 탐지
- Gap 중심을 Pure Pursuit 스타일로 조향 변환

---

## 🔶 MPPI (Model Predictive Path Integral)

MPPI는 샘플 기반 최적화 컨트롤러로

여러 개의 candidate trajectory를 생성하고 비용(cost)을 비교해

가장 안전하고 빠른 조향·속도를 선택합니다.

본 프로젝트에서 사용된 MPPI 특징:

- 랜덤 샘플 기반 trajectory rollout
- 충돌 비용, 경로 이탈 비용, 조향 변화 비용 사용
- FGM보다 더 공격적·최적화된 고속 주행 가능
- ROS2 timer 기반 실시간 MPC 동작
