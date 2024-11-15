// RAMPS 1.4 핀 정의
#define X_STEP_PIN      54
#define X_DIR_PIN       55
#define X_ENABLE_PIN    38
#define Y_STEP_PIN      60
#define Y_DIR_PIN       61
#define Y_ENABLE_PIN    56
#define X_MIN_PIN       3   // X min 엔드스탑 핀
#define Y_MAX_PIN       15  // Y max 엔드스탑 핀

void setup() {
  Serial.begin(115200); // 시리얼 모니터 초기화

  // 모터 제어 핀 초기화
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  // 엔드스탑 핀 초기화
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);

  // 모터 활성화 (활성화 상태는 LOW)
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);

  // 초기 지연 시간
  delay(1000);
}

void loop() {
  // 1. X축 모터를 시계 방향으로 회전시키며 X min 엔드스탑이 눌릴 때까지 이동
  Serial.println("Moving X axis clockwise until X MIN endstop is triggered...");
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(X_DIR_PIN, LOW); // 시계 방향 설정 (반대 방향으로 설정할 수도 있음)
  
  while (digitalRead(X_MIN_PIN) == HIGH) { // 엔드스탑이 눌리면 LOW로 전환
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(100);
  }

  // X축 모터 정지
  Serial.println("X MIN endstop triggered. Stopping X axis motor...");
  delay(1000);

  // 2. X축 모터를 반시계 방향으로 1600스텝 이동
  Serial.println("Moving X axis counterclockwise for 1600 steps...");
  digitalWrite(X_DIR_PIN, HIGH); // 반시계 방향 설정
  
  for (int i = 0; i < 5895; i++) { // 1600스텝 이동
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(100);
  }

  // X축 모터 정지 후 잠시 대기
  Serial.println("Completed 1600 steps counterclockwise on X axis.");
  delay(1000);

  // 3. Y축 모터를 오른쪽으로 이동하며 Y max 엔드스탑이 눌릴 때까지 이동
  Serial.println("Moving Y axis to the right until Y MAX endstop is triggered...");
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Y_DIR_PIN, HIGH); // Y축 방향을 반대로 설정
  
  while (digitalRead(Y_MAX_PIN) == HIGH) { // Y max 엔드스탑이 눌리면 LOW
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(100);
  }

  // Y축 모터 정지
  Serial.println("Y MAX endstop triggered. Stopping Y axis motor...");
  delay(1000);

  // 모든 동작 완료 후 다음 단계를 Signal하기 위해 시리얼로 메시지 전송
  Serial.println("Homing process completed.");

  // 무한 대기하여 반복되지 않도록 함
  while (1);
}
