# Practice 1 학습 내용

## 1. 실습 목표
- ROS2의 기본 노드 구조와 토픽 통신 개념을 이해한다.
- `turtlesim` 패키지를 사용하여 키보드로 거북이를 조작하고, 노드 간의 데이터 흐름을 확인한다.

---
## 2. 학습 내용
  
- **작성자** : 윤우린
- **작성일** : 2025-09-23
- **1주 차: ROS2 개발 환경 구축 및 기본 개념 학습 및 Turtlesim 실습**
- 목표: ROS2 개발을 위한 필수 환경을 세팅하고, ROS2의 핵심 작동 원리를 이해 & Turtlesim 시뮬레이 활용 기초 실습

**1. 개발 환경 구축:** Ubuntu 22.04(Jammy Jellyfish)에 ROS2 Humble Hawksbill 버전을 설치합니다. 이 과정에서 ROS2 공식 문서의 설치 가이드를 참고하여 의존성 패키지 설치, 로케일 설정, ROS2 패키지 다운로드 및 환경 설정 스크립트 소싱을 완료했습니다.

**2. ROS2 기본 개념:**
* 노드(Nodes): ROS2의 최소 실행 단위로, 특정 기능을 수행하는 프로세스입니다. 예를 들어, 로봇의 센서 데이터를 읽는 노드나 모터를 제어하는 노드 등이 있습니다.

* 토픽(Topics): 노드들이 데이터를 주고받는 통신 채널입니다. 센서 데이터, 제어 명령 등 다양한 형태의 메시지를 주고받을 수 있습니다.

* 메시지(Messages): 토픽을 통해 전송되는 실제 데이터 형식입니다. ROS2는 다양한 표준 메시지 타입을 제공하며, 필요에 따라 사용자 정의 메시지를 만들 수도 있습니다.

* 패키지(Packages): 노드, 메시지, 설정 파일 등 특정 기능을 수행하는 데 필요한 모든 것을 묶어 놓은 단위입니다. 프로젝트는 여러 패키지로 구성될 수 있습니다.
  
**3. Turtlesim 시뮬레이터 실습:**

* turtlesim_node를 실행하여 거북이 시뮬레이터를 띄우고, turtle_teleop_key 노드를 실행하여 키보드로 거북이를 움직여 보았습니다.

* ROS2 node list와 같은 ROS2 CLI를 사용하여 현재 실행 중인 노드들을 확인했습니다.

* ROS2 topic list와 ROS2 topic info 명령어를 통해 거북이의 위치 정보를 주고받는 /turtle1/pose 토픽과 거북이의 이동 명령을 받는 /turtle1/cmd_vel 토픽의 존재를 확인하고, 각각의 메시지 타입을 살펴보았습니다.

---
## 3. 실습 결과

  **1. 이태웅** (09/23)
<figure>
    <img src="https://i.esdrop.com/d/f/TofYJ3q0s2/dwU2Clnpeu.png" title="1주차 실습">    
    <figcaption>1주차 실습</figcaption>
</figure>

---
**2. 남지연** (09/23)

<img width="460" height="479" alt="image" src="https://github.com/user-attachments/assets/0cb5594b-7e10-4c9f-a008-9c11dfe75968" />

---
**3. 윤우린** (09/23)

<img width="717" height="702" alt="Image" src="https://github.com/user-attachments/assets/b3f0a9d3-1b9e-4654-9187-175b5f118bcb" />" />

---
