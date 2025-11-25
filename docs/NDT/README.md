[ㅁㅁㅁ 2b66d47cdeb3804e9c6bcf1ef20da3a1.md](https://github.com/user-attachments/files/23736019/2b66d47cdeb3804e9c6bcf1ef20da3a1.md)
# ㅁㅁㅁ

Status: 시작 전
프로젝트: cs스터디 (https://www.notion.so/cs-1016d47cdeb380b3a610deccac7d6634?pvs=21)

---

# **NDT  기반 Localization**

---

# **NDT란?**

- NDT(Normal Distributions Transform)는 LiDAR 포인트 클라우드를 확률적 분포(정규분포, Gaussian) 로 표현하고, Scan을 기존 맵과 확률적 매칭하여 현재 위치를 추정하는 Localization 알고리즘
- 기존 지도(Map)를 셀(Cell) 단위로 나누고 각 셀 안의 포인트를 정규분포(평균, 공분산)로 모델링한 뒤 새로운 Scan이 들어올 때 가장 잘 매치되는 위치를 찾아 로봇의 Pose(x, y, yaw) 를 추정

---

# NDT의 특징

### 1. ICP보다 더 안정적

ICP는 포인트 간의 최근접 매칭에 의존 → 노이즈에 약함

NDT는 확률적 분포 기반 → Outlier에 강함, 수렴 안정적

### 2. LiDAR 기반 Localization에 많이 사용됨

자율주행차량, 모바일 로봇에서 실시간 Localization에 많이 채택됨

### 3. 고정된 맵(Local map)을 미리 가우시안 분포로 변환하여 계산량 감소

→ 빠른 매칭이 가능하며 Global Localization도 가능

---

# **NDT Localization 원리**

## (1) **맵을 Grid로 나누기**

- 각 셀마다 point들 모으기

## (2) **셀마다 정규분포 모델 생성**

- 각 셀에 대해 평균과 공분산을 계산하여 **3차원 Gaussian Distribution**을 만듦.

## (3) **새로운 LiDAR Scan 입력**

스캔 포인트들을 맵에 투영하면서 각 포인트가 어떤 셀의 정규분포에 잘 들어맞는지 계산

**⇒** 확률이 가장 높은 pose로 이동시키는 방향을 Gradient 기반으로 업데이트.

## (4) **최적화(Optimization)**

목표:현재 scan이 map의 Gaussian 분포들에 가장 잘 맞는 pose 찾은 후 반복적으로 보정

---

# **NDT Localization의 장점**

### 1. Outlier·노이즈에 강함

개별 포인트를 직접 비교방식 보다 안정적

### 2. 수렴이 빠른 경우가 많음

Gaussian 기반 cost function이 smooth

### 3. sparse point cloud에서도 잘 동작

### 4. Global matching 가능

초기 pose가 조금 틀려 있어도 잘 맞춰짐

---

# 6️⃣ **단점**

### 1. 계산량이 ICP보다 많을 수 있음

특히 map이 크면 Gaussian 셀 계산이 무거움

### 2. resolution 민감

grid 사이즈를 잘못 선택하면 성능 저하

### 3.  dynamic object에 취약

움직이는 차, 사람 등은 Gaussian 모델을 어지럽힘

---
