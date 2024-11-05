import math

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

def get_user_input():
    L1 = float(input("Enter the length L1: "))
    L2 = float(input("Enter the distance L2: "))
    target_x = float(input("Enter the target x: "))
    target_y = float(input("Enter the target y: "))
    offset = float(input("Enter the offset: "))
    
    return L1, L2, target_x, target_y, offset

def main():
    L1, L2, target_x, target_y, offset = get_user_input()
    
    theta_a, theta_b = inverse_kinematics(L1, L2, offset, target_x, target_y)
    
    print(f"theta_a: {theta_a:.2f}, theta_b: {theta_b:.2f}")

if __name__ == "__main__":
    main()
