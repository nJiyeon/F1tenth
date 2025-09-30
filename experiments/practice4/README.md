# Practice 4: Pure Pursuit 기반 주행 알고리즘 실습
<img width="579" height="189" alt="image" src="https://github.com/user-attachments/assets/502c0c52-f202-4bc9-81ae-5c2846768387" />


<br><br>

## 1. 실습 목적
- 경로 추종(Path Tracking)을 위한 Pure Pursuit 알고리즘을 이해하고 구현한다.
- **L 값(look ahead distance)** 의 의미와 역할을 학습하고, 주행 안정성에 적합한 값을 직접 실험을 통해 찾는다.
- Advanced 단계에서는 곡률(Curvature)에 따라 속도를 조정하는 동적 주행 기법을 실습한다.
  <br><br><br>

## 2. 실습 과제
- Pure Pursuit 알고리즘을 ROS2 Python 노드로 구현한다.
- **L 값(look ahead distance)** 을 변화시키며 주행 성능(곡선 추종, 직선 안정성 등)을 비교·분석한다.
- Advanced: 곡률값(곡선의 정도)에 따라 속도를 자동으로 조절하는 기능을 추가하여 주행 효율성을 높인다.
  <br><br>

## 3. 제출 방식
- `자기이름.md` 파일에 실습 과정(알고리즘 구현, L 값 조정 실험, Advanced 기능 구현) 기록
- `자기이름.py` 파일에 각자 결과(시뮬레이션 또는 실제 주행 결과) 이미지 첨부
- 제출 기간 : ~ 10/10 (금)
