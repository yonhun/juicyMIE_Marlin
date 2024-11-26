import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
L1 = 90    # 첫 번째 링크 길이
L2 = 113   # 두 번째 링크 길이
D = 44     # 두 모터 간의 거리
x0 = 38.0  # 첫 번째 모터의 X 좌표
y0 = -48.45  # 모터의 Y 좌표

# 고정된 모터 위치
P1 = np.array([x0, y0])
P2 = np.array([x0 + D, y0])

# 모터 각도 범위 설정 (도 단위)
a_start_deg = 45    # 모터 1 각도의 최소값
a_end_deg = 250     # 모터 1 각도의 최대값
b_start_deg = -75   # 모터 2 각도의 최소값
b_end_deg = 135     # 모터 2 각도의 최대값

# 각도 증가 단위
a_step_deg = 1     # 모터 1 각도의 증가 단위
b_step_deg = 1     # 모터 2 각도의 증가 단위

# 모터 1 각도 리스트 생성
a_deg_values = np.arange(a_start_deg, a_end_deg + a_step_deg, a_step_deg)
# 모터 2 각도 리스트 생성
b_deg_values = np.arange(b_start_deg, b_end_deg + b_step_deg, b_step_deg)

# 모든 프레임의 각도 조합 생성
angle_pairs = []
for a_deg in a_deg_values:
    for b_deg in b_deg_values:
        # 각도를 라디안으로 변환
        a_rad = np.deg2rad(a_deg)
        b_rad = np.deg2rad(b_deg)
        
        # 첫 번째 링크 끝점 좌표 계산 (모터 1)
        x1 = P1[0] + L1 * np.cos(a_rad)
        y1 = P1[1] + L1 * np.sin(a_rad)

        # 두 번째 링크 끝점 좌표 계산 (모터 2)
        x2 = P2[0] + L1 * np.cos(b_rad)
        y2 = P2[1] + L1 * np.sin(b_rad)

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
            continue  # 유효하지 않은 경우 다음 모터 2 각도로
        h = np.sqrt(h_sq)

        # 두 링크 끝점을 연결하는 선에 수직인 단위 벡터 계산
        nx = (y1 - y2) / d
        ny = (x2 - x1) / d

        # 엔드이펙터의 두 가지 위치 계산 (상하 대칭)
        ex1 = mid_x + h * nx
        ey1 = mid_y + h * ny
        ex2 = mid_x - h * nx
        ey2 = mid_y - h * ny

        # 엔드이펙터 위치 선택 (여기서는 ex1, ey1 사용)
        ex, ey = ex1, ey1

        # y0 이하의 위치 제외
        if ey <= y0:
            continue  # y가 -48.5 이하이면 제외

        # 모터 1과 엔드이펙터 사이의 거리 계산
        dist_P1_to_EE = np.hypot(ex - P1[0], ey - P1[1])
        # 모터 2와 엔드이펙터 사이의 거리 계산
        dist_P2_to_EE = np.hypot(ex - P2[0], ey - P2[1])

        # 거리 조건 체크: 거리 >= L1 + L2 - 0.1 이면 반복 중단
        if dist_P1_to_EE >= (L1 + L2 - 0.1) or dist_P2_to_EE >= (L1 + L2 - 0.1):
            break  # 현재 모터 1 각도에서 모터 2 각도의 반복 중단

        # 각도 쌍과 엔드이펙터 위치를 리스트에 추가
        angle_pairs.append((a_deg, b_deg, ex, ey))

# 총 프레임 수
num_frames = len(angle_pairs)

# 이전 모터 1 각도를 저장할 전역 변수 초기화
prev_a_deg = None

# 애니메이션을 위한 그림과 축 설정
fig, ax = plt.subplots(figsize=(8, 8))

# 여유 공간을 위한 마진 값 설정
margin_x = 50  # x축 마진
margin_y = 50  # y축 마진

# 축 범위 설정 시 마진을 적용
ax.set_xlim(x0 - L1 - L2 - margin_x, x0 + D + L1 + L2 + margin_x)
ax.set_ylim(-150 , y0 + L1 + L2 + margin_y)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('PARALLEL SCARA WORKSPACE')
ax.set_xlabel('X')
ax.set_ylabel('Y')

# 로봇 링크를 나타낼 선분과 엔드이펙터를 나타낼 점 생성
link1_line, = ax.plot([], [], 'o-', lw=2, color='blue', label='Link 1')
link2_line, = ax.plot([], [], 'o-', lw=2, color='green', label='Link 2')
end_effector_point, = ax.plot([], [], 'ro', markersize=5, label='End Effector')
trajectory_line, = ax.plot([], [], 'r-', lw=1, label='End Effector Path')
ax.legend()

# 엔드이펙터 위치를 저장할 리스트 초기화
end_effector_x = []
end_effector_y = []

# 애니메이션 초기화 함수
def init():
    link1_line.set_data([], [])
    link2_line.set_data([], [])
    end_effector_point.set_data([], [])
    trajectory_line.set_data([], [])
    return link1_line, link2_line, end_effector_point, trajectory_line

# 애니메이션 업데이트 함수
def update(frame):
    global prev_a_deg  # 전역 변수 선언

    # 현재 프레임의 모터 각도와 엔드이펙터 위치
    a_deg, b_deg, ex, ey = angle_pairs[frame % num_frames]
    a_rad = np.deg2rad(a_deg)
    b_rad = np.deg2rad(b_deg)

    # 모터 1의 각도 변경 감지
    if prev_a_deg is not None and a_deg != prev_a_deg:
        # 엔드이펙터 경로 리스트에 NaN 추가하여 선을 끊음
        end_effector_x.append(np.nan)
        end_effector_y.append(np.nan)

    prev_a_deg = a_deg  # 현재 모터 1 각도를 저장

    # 첫 번째 링크 끝점 좌표 계산 (모터 1)
    x1 = P1[0] + L1 * np.cos(a_rad)
    y1 = P1[1] + L1 * np.sin(a_rad)

    # 두 번째 링크 끝점 좌표 계산 (모터 2)
    x2 = P2[0] + L1 * np.cos(b_rad)
    y2 = P2[1] + L1 * np.sin(b_rad)

    # 엔드이펙터 위치를 리스트에 추가
    end_effector_x.append(ex)
    end_effector_y.append(ey)

    # 링크1의 좌표 설정
    link1_x = [P1[0], x1, ex]
    link1_y = [P1[1], y1, ey]
    link1_line.set_data(link1_x, link1_y)

    # 링크2의 좌표 설정
    link2_x = [P2[0], x2, ex]
    link2_y = [P2[1], y2, ey]
    link2_line.set_data(link2_x, link2_y)

    # 엔드이펙터 위치 설정
    end_effector_point.set_data(ex, ey)

    # 엔드이펙터 경로 업데이트
    trajectory_line.set_data(end_effector_x, end_effector_y)

    return link1_line, link2_line, end_effector_point, trajectory_line

# 애니메이션 생성
ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True, interval=1)

# 애니메이션 표시
plt.show()