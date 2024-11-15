import subprocess
import serial
import time

# Arduino CLI 경로 및 보드 정보 설정
arduino_cli_path = "arduino-cli"
fqbn_mega = "arduino:avr:mega"
port = "/dev/ttyUSB0"
baud_rate = 115200

# 프로젝트 경로 설정
homing_path = "/home/ssl/juicyMIE_Marlin/homing"
marlin_path = "/home/ssl/juicyMIE_Marlin/Marlin-2.1.2.4/Marlin"

def compile_and_upload(sketch_path, sketch_name):
    try:
        # 컴파일
        print(f"컴파일 중: {sketch_name}")
        compile_command = [
            arduino_cli_path, "compile", "--fqbn", fqbn_mega, sketch_path
        ]
        subprocess.run(compile_command, check=True)
        
        # 업로드
        print(f"업로드 중: {sketch_name}")
        upload_command = [
            arduino_cli_path, "upload", "-p", port, "--fqbn", fqbn_mega, sketch_path
        ]
        subprocess.run(upload_command, check=True)
        
        print(f"{sketch_name} 업로드 완료!")
    
    except subprocess.CalledProcessError as e:
        print(f"오류 발생: {sketch_name} 업로드 실패. 오류: {e}")

def monitor_homing_complete():
    """ homing.ino가 완료될 때까지 Serial로 모니터링 """
    try:
        # Serial 통신 시작
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print("Homing 완료를 기다리는 중...")
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    print(line)
                    # 완료 메시지를 확인하면 함수 종료
                    if "Homing process completed." in line:
                        print("Homing 완료 감지!")
                        break
                time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Serial 연결 실패: {e}")

# 첫 번째 파일 업로드 (homing.ino)
compile_and_upload(homing_path, "homing.ino")

# Homing 완료 대기
monitor_homing_complete()

# 두 번째 파일 업로드 (Marlin 펌웨어)
compile_and_upload(marlin_path, "Marlin 펌웨어")
