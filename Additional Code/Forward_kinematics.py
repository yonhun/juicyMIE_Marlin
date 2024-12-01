import math

def forward_kinematics(L1, L2, theta1, theta2, offset):
    # 각도를 라디안으로 변환
    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)

    # 두 점 (x1, y1), (x2, y2)의 좌표 계산
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = L1 * math.cos(theta2) + offset
    y2 = L1 * math.sin(theta2)

    # 두 점 사이의 중점 계산
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2

    # 두 점 사이의 거리 계산
    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # 중점에서 목표 점까지의 거리 계산
    h = math.sqrt(L2**2 - (d / 2)**2)

    # 두 점을 잇는 벡터에 수직인 단위 벡터 계산
    if d != 0:
        nx = (y1 - y2) / d
        ny = (x2 - x1) / d
    else:
        # 두 점이 같은 경우, 임의의 수직 벡터 선택
        nx, ny = 0, 1

    # 목표 점 계산
    target_x = mid_x + h * nx
    target_y = mid_y + h * ny

    return x1, y1, x2, y2, target_x, target_y

# 사용자 입력을 받는 함수
def get_user_input():
    L1 = float(input("Enter the length L1: "))
    L2 = float(input("Enter the distance L2: "))
    theta1 = float(input("Enter the angle theta1 (in degrees): "))
    theta2 = float(input("Enter the angle theta2 (in degrees): "))
    offset = float(input("Enter the offset between two motors: "))
    offset_x = float(input("Enter the scara offset X: "))
    offset_y = float(input("Enter the scara offset Y: "))
    
    return L1, L2, theta1, theta2, offset, offset_x, offset_y

# 메인 함수
def main():
    # 사용자로부터 입력 받기
    L1, L2, theta1, theta2, offset, offset_x, offset_y = get_user_input()
    
    # forward kinematics 계산
    x1, y1, x2, y2, target_x, target_y = forward_kinematics(L1, L2, theta1, theta2, offset)
    
    # 결과 출력
    print(f"Point 1 (x1, y1): ({x1:.2f}, {y1:.2f})")
    print(f"Point 2 (x2, y2): ({x2:.2f}, {y2:.2f})")
    print(f"Target Point (x, y) at distance L2 from both points: ({target_x:.2f}, {target_y:.2f})")
    print(f"Real Target Point (x, y): ({target_x + offset_x}, {target_y + offset_y}) ")

# 프로그램 실행
if __name__ == "__main__":
    main()
