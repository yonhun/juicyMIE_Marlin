# 🖨️ juicyMIE_Marlin - 휴대용 5절 링크 3D 프린터 펌웨어 🚀

**juicyMIE_Marlin**은 기존 직교 좌표계 3D 프린터의 구조적 한계를 벗어나, 휴대성을 극대화하기 위해 설계된 **5절 링크(Parallel SCARA) 구조** 기반의 맞춤형 Marlin 펌웨어입니다. 전용 모바일 앱과 연동하여 블루투스로 통신하며, 휴대용 소형 FDM 3D 프린터 제어를 목적으로 개발되었습니다.

---

## 🚀 프로젝트 데모

### 📸 완성품 사진

<div align="center">
  <img src="![Image](https://github.com/user-attachments/assets/fe483332-f249-406a-a849-cb694a7c4d55)" alt="3D 프린터 완성품 사진 1" width="400">
  <img src="![Image](https://github.com/user-attachments/assets/95c2282f-d2ef-41c2-a2b0-59c948922026)" alt="3D 프린터 완성품 사진 2" width="400">
</div>

### 🎬 프린터 동작 및 출력 데모

<div align="center">
  <img src="![Image](https://github.com/user-attachments/assets/320dcbbf-8b21-4be0-ac9a-d248d5ec650b)" alt="프린터 동작 데모" width="400">
</div>

---

## ⚙️ 기구학 메커니즘 (Kinematic Mechanism)

Marlin의 코어 모션 제어부에 5절 링크 시스템을 위한 정기구학(Forward Kinematics) 및 역기구학(Inverse Kinematics) 수식을 적용했습니다.

### 1. 정기구학 (Forward Kinematics)

<div align="center">
  <img src="![Image](https://github.com/user-attachments/assets/60c27aff-8800-4dc0-b3f3-eab6ce08d27e)" alt="정기구학 다이어그램" width="600">
</div>

정기구학의 경우 모터 A, B의 좌표인 $P_A$, $P_B$ 그리고 두 모터 사이의 거리인 $offset$, 각 모터에 주어진 각도인 $\theta_A$, $\theta_B$, 그리고 첫번째, 두번째 링크의 길이인 $L_1, L_2$가 주어졌을 때 이를 이용하여 End-Effector의 좌표인 $P_E$를 구하는 과정입니다.

**1.1 모터 A에 결합된 첫번째 링크의 끝점 $P_1$, 모터 B에 결합된 첫번째 링크의 끝점 $P_2$ 좌표 구하기**
> $P_1 = (L_1\cos\theta_A, L_1\sin\theta_A)$
> $P_2 = (offset + L_1\cos\theta_B, L_1\sin\theta_B)$

**1.2 $P_1$과 $P_2$의 중점인 $P_C$ 구하기**
> $P_C = \frac{P_1 + P_2}{2} = \left(\frac{x_1 + x_2}{2}, \frac{y_1 + y_2}{2}\right)$

**1.3 $P_C$ 점을 지나면서 벡터 $\vec{P_1P_2}$에 직교하는 벡터 $n$ 구하기**
거리 $d$: $d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$
단위 벡터 $n_{unit}$: $n_{unit} = \left(\frac{y_1 - y_2}{d}, \frac{x_1 - x_2}{d}\right)$
법선 벡터의 크기 $|n|$: $|n| = \sqrt{{L_2}^2 - \left(\frac{d}{2}\right)^2}$
최종 벡터 $n$: $n = \sqrt{{L_2}^2 - \left(\frac{d}{2}\right)^2} \times \left(\frac{y_1 - y_2}{d}, \frac{x_1 - x_2}{d}\right)$

**1.4 End-Effector의 좌표 $P_E$ 구하기**
> $P_E = P_C + n = P_C + \sqrt{{L_2}^2 - \left(\frac{d}{2}\right)^2} \times \left(\frac{y_1 - y_2}{d}, \frac{x_1 - x_2}{d}\right)$


### 2. 역기구학 (Inverse Kinematics)

<div align="center">
  <img src="![Image](https://github.com/user-attachments/assets/9bf52db3-ac38-4127-88c3-4f40e751b8fd)" alt="역기구학 다이어그램" width="600">
</div>

역기구학의 경우 모터 A, B의 좌표인 $P_A$, $P_B$ 그리고 두 모터 사이의 거리인 $offset$, 첫번째, 두번째 링크의 길이인 $L_1, L_2$와 목표하는 End-Effector 의 좌표인 $P_E(x_E, y_E)$가 주어졌을 때 이를 이용하여 각 모터가 회전해야 할 각도인 $\theta_A, \theta_B$를 구하는 과정입니다.

**2.1 원점과 모터에서 $P_E$까지의 거리 $a, b$ 구하기**
> $a = \sqrt{{x_E}^2 + {y_E}^2}$
> $b = \sqrt{(x_E - offset)^2 + {y_E}^2}$

**2.2 1번 삼각형의 각 변의 길이인 $a, b, offset$과 제2코사인법칙을 이용하여 $\alpha, \gamma$ 구하기**
> $b^2 = a^2 + offset^2 - 2a \times offset \times \cos\alpha \Rightarrow \cos\alpha = \frac{a^2 + offset^2 - b^2}{2a \times offset}$
> $\therefore \alpha = \cos^{-1}\left(\frac{a^2 + offset^2 - b^2}{2a \times offset}\right)$
> 
> $a^2 = b^2 + offset^2 - 2b \times offset \times \cos\gamma \Rightarrow \cos\gamma = \frac{b^2 + offset^2 - a^2}{2b \times offset}$
> $\therefore \gamma = \cos^{-1}\left(\frac{b^2 + offset^2 - a^2}{2b \times offset}\right)$

**2.3 2번 삼각형의 각 변의 길이인 $L_1, L_2, a$와 제2코사인법칙을 이용하여 $\beta$ 구하기**
> ${L_2}^2 = {L_1}^2 + a^2 - 2a \times L_1 \times \cos\beta \Rightarrow \cos\beta = \frac{{L_1}^2 + a^2 - {L_2}^2}{2aL_1}$
> $\therefore \beta = \cos^{-1}\left(\frac{{L_1}^2 + a^2 - {L_2}^2}{2aL_1}\right)$

**2.4 3번 삼각형의 각 변의 길이인 $L_1, L_2, b$와 제2코사인법칙을 이용하여 $\delta$ 구하기**
> ${L_2}^2 = {L_1}^2 + b^2 - 2b \times L_1 \times \cos\delta \Rightarrow \cos\delta = \frac{{L_1}^2 + b^2 - {L_2}^2}{2bL_1}$
> $\therefore \delta = \cos^{-1}\left(\frac{{L_1}^2 + b^2 - {L_2}^2}{2bL_1}\right)$

**2.5 최종 모터 각도 $\theta_A, \theta_B$ 구하기**
> $\theta_A = \alpha + \beta = \cos^{-1}\left(\frac{a^2 + offset^2 - b^2}{2a \times offset}\right) + \cos^{-1}\left(\frac{{L_1}^2 + a^2 - {L_2}^2}{2aL_1}\right)$
> $\theta_B = \gamma + \delta = \cos^{-1}\left(\frac{b^2 + offset^2 - a^2}{2b \times offset}\right) + \cos^{-1}\left(\frac{{L_1}^2 + b^2 - {L_2}^2}{2bL_1}\right)$


### 3. 기구학 검증 및 작업 공간 (Workspace)

<div align="center">
  <img src="![Image](https://github.com/user-attachments/assets/2bcf953b-4bce-4120-9e74-171f93867b4d)" alt="기구학 검증 및 작업 공간 시뮬레이션" width="600">
  <p><i>5절 링크 기반 End-effector가 도달할 수 있는 작업 공간(Workspace) 시뮬레이션 결과</i></p>
</div>

---

## 🛠️ Marlin C 코드 및 설정 변경 사항

본 프로젝트를 위해 원본 Marlin Firmware에서 5절 링크 메커니즘을 구동할 수 있도록 다음 부분들이 수정되었습니다.

### 1. C Code 변경 (`src/module/` )
- **기구학 제어 로직 교체**: 
  - 기존 직교좌표계(Cartesian) 수식을 5절 링크에 맞춘 **Parallel SCARA 역기구학 수식**으로 전면 교체하여 목표 좌표에 따른 모터 회전 각도 계산 알고리즘 적용.
- **작업 공간 검사(Workspace Check) 알고리즘 추가**:
  - 5절 링크의 물리적 구동 범위를 벗어나는 좌표로 이동을 시도할 경우, 수식의 허수를 방지하고 하드웨어 파손을 막기 위한 예외 처리 로직 구현.

### 2. Configuration 변경 (`Configuration.h` & `Configuration_adv.h`)
- **기구학 파라미터 정의**: 5절 링크 구조 제어를 위한 매크로 활성화 및 주요 링크 길이($L_1, L_2$)와 모터 간 간격(offset) 설정.
- **작업 공간(Workspace) 제약 설정**: 베드 사이즈 및 모터의 구동 가능 반경을 고려한 소프트웨어 엔드스탑 제약 설정 (`X_MIN_POS`, `X_MAX_POS` 등).
- **모터 및 센서 설정**: X, Y 모터의 회전 방향 및 원점(Homing) 위치에 맞춘 엔드스탑 로직 튜닝 (`INVERT_X_DIR`, `USE_XMIN_PLUG` 등).
- **스텝 값 계산**: `DEFAULT_AXIS_STEPS_PER_UNIT`을 직진 운동 스텝이 아닌 각도(Degree)당 필요 스텝 수로 변경 적용.

---

## 💻 개발 및 빌드 환경 설정

이 펌웨어를 보드에 업로드하려면 로컬 환경에서 다음 단계를 따르세요:

1. **VSCode** 및 **PlatformIO** 확장 프로그램 설치
2. 저장소 클론
   ```bash
   git clone [https://github.com/yonhun/juicyMIE_Marlin.git](https://github.com/yonhun/juicyMIE_Marlin.git)
   ```
3. VSCode에서 juicyMIE_Marlin 폴더 열기
4. platformio.ini 파일에서 사용 중인 3D 프린터 메인보드에 맞게 default_envs 확인 및 수정
5. PlatformIO 탭에서 Build 클릭하여 컴파일
6. 보드를 연결하고 Upload 버튼을 눌러 펌웨어 플래싱

---

## 📞 문의 및 연관 리포지토리
* **Email:** [0hoo2@naver.com](mailto:0hoo2@naver.com)
* **App Repository:** [juicyMIE](https://github.com/Githarold/juicyMIE) (휴대용 3D 프린터 제어용 Flutter 모바일 앱)

---
