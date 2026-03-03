import numpy as np
import matplotlib.pyplot as plt

# Parameters
L1 = 90   # 첫 번째 링크 길이
L2 = 113  # 두 번째 링크 길이
D = 44    # 두 모터 간의 거리
x0 = 38.0   # 첫 번째 모터의 X 좌표
y0 = -48.45 # 모터의 Y 좌표

# 고정된 모터 위치
P1 = np.array([x0, y0])
P2 = np.array([x0 + D, y0])

# 모터 1 각도 리스트 생성 (도 단위)
a_deg_values = np.arange(45, 251, 1)
# 모터 2 각도 리스트 생성 (도 단위)
b_deg_values = np.arange(-75, 136, 1)

# 엔드이펙터 위치를 저장할 리스트 초기화
final_x = []
final_y = []

# 각 모터 1 각도에 대해 반복
for a_deg in a_deg_values:
    for b_deg in b_deg_values:
        # 각도를 라디안으로 변환
        a_rad = np.deg2rad(a_deg)
        b_rad = np.deg2rad(b_deg)

        # 첫 번째 링크 끝점의 좌표 계산 (모터 위치 포함)
        x1 = x0 + L1 * np.cos(a_rad)
        y1 = y0 + L1 * np.sin(a_rad)

        x2 = x0 + D + L1 * np.cos(b_rad)
        y2 = y0 + L1 * np.sin(b_rad)

        # 두 링크 끝점의 중점 계산
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        # 두 링크 끝점 사이의 거리 계산
        dx = x2 - x1
        dy = y2 - y1
        d = np.hypot(dx, dy)

        # 중점에서 엔드이펙터까지의 거리 계산 (h)
        h_sq = L2**2 - (d / 2)**2
        if h_sq < 0 or d == 0:
            continue  # 다음 모터 2 각도로
        h = np.sqrt(h_sq)

        # 두 링크 끝점을 연결하는 선에 수직인 단위 벡터 계산
        nx = (y1 - y2) / d
        ny = (x2 - x1) / d

        # 엔드이펙터의 두 가지 위치 계산 (상하 대칭)
        ex_list = [mid_x + h * nx, mid_x - h * nx]
        ey_list = [mid_y + h * ny, mid_y - h * ny]

        # 두 위치에 대해 각각 처리
        condition_met = False
        for ex, ey in zip(ex_list, ey_list):
            # y 좌표가 -48.5 미만인 경우 제외
            if ey < -48.5:
                continue  # 다음 위치로

            # 거리 조건 계산
            dist_P1_to_EE = np.hypot(ex - P1[0], ey - P1[1])
            dist_P2_to_EE = np.hypot(ex - P2[0], ey - P2[1])
            max_distance = L1 + L2 - 0.1

            if dist_P1_to_EE >= max_distance or dist_P2_to_EE >= max_distance:
                condition_met = True
                break  # 현재 모터 1 각도에서 모터 2 각도의 반복 중단

            # 유효한 엔드이펙터 위치 저장
            final_x.append(ex)
            final_y.append(ey)

        if condition_met:
            break  # 거리 조건을 만족하지 않으면 모터 2 각도 반복 중단

# 작업 공간 플롯
plt.figure(figsize=(8, 8))
plt.plot(final_x, final_y, 'k.', markersize=1, label='Reachable Workspace')

# 원하는 점들을 정의
specific_points = np.array([
    [38, -48.45], # 모터 1번 위치
    [82, -48.45], # 모터 2번 위치
    [-15, -15],   # 실제 0,0 위치
    [-15, 135],   # 실제 0, 150 위치
    [135, -15],   # 실제 150, 0 위치
    [135, 135]    # 실제 150, 150 위치
])

# 원하는 점들을 플롯
plt.plot(specific_points[:, 0], specific_points[:, 1], 'ro', markersize=5, label='Specific Points')

plt.axis('equal')
plt.title('PARALLEL SCARA WORKSPACE')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.legend()
plt.show()
