import math


def get_user_input():
    theta_a = float(input("Enter the angle theta_a: "))
    theta_b = float(input("Enter the angle theta_b: "))
    return theta_a, theta_b


def forward_kinematics(L1, L2, theta_a, theta_b, offset):
    # 각도를 라디안으로 변환
    theta_a = math.radians(theta_a)
    theta_b = math.radians(theta_b)

    # 두 점 (x1, y1), (x2, y2)의 좌표 계산
    x1 = L1 * math.cos(theta_a)
    y1 = L1 * math.sin(theta_a)

    x2 = L1 * math.cos(theta_b) + offset
    y2 = L1 * math.sin(theta_b)

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

    return target_x, target_y


def inverse_kinematics(L1, L2, offset, target_x, target_y):

    a = math.sqrt(target_x**2 + target_y**2)
    b = math.sqrt((target_x-offset)**2 + target_y**2)

    alpha = math.acos((a**2 + offset**2 - b**2) / (2*a*offset))
    gamma = math.acos((b**2 + offset**2 - a**2) / (2*b*offset))

    beta = math.acos((L1**2 + a**2 - L2**2) / (2*a*L1))
    delta = math.acos((L1**2 + b**2 - L2**2) / (2*b*L1))

    theta_a = math.degrees(alpha + beta)
    theta_b = math.degrees(math.pi - gamma - delta)

    return theta_a, theta_b

def main():
    L1 = 90
    L2 = 113
    offset = 44
    tolerance = 0.01

    theta_a, theta_b = get_user_input()

    target_x, target_y = forward_kinematics(L1, L2, theta_a, theta_b, offset)
    print(f"target_x: {target_x}, target_y: {target_y}")

    if target_y >= 3:
        theta_a_inv, theta_b_inv = inverse_kinematics(L1, L2, offset, target_x, target_y)
        print(f"theta_a_inv: {theta_a_inv}, theta_b_inv: {theta_b_inv}")

        if abs(theta_a_inv - theta_a) <= tolerance and abs(theta_b_inv - theta_b) <= tolerance:
            print("Forward and inverse kinematics are consistent.")
        else:
            print("Forward and inverse kinematics are not consistent.")
    
    else:
        print("target_y is less than 3")
   

if __name__ == "__main__":
    main()
