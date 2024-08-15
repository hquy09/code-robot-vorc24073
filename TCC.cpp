#include <Servo.h> // Thư viện Servo
#include <PS2X_lib.h> // Thư viện PS2X
#include <Wire.h> // Thư viện I2C
#include <Adafruit_PWMServoDriver.h> // Thư viện điều khiển động cơ
PS2X ps2x; // Tạo đối tượng PS2X
// Khai báo các chân kết nối PS2
#define PS2_DAT 12  // Chân DATA
#define PS2_CMD 11  // Chân COMMAND
#define PS2_SEL 10  // Chân ATTENTION
#define PS2_CLK 13  // Chân CLOCK

// Khai báo các chân kết nối servo và motor
#define SERVO_1_PIN 2 // Chân điều khiển servo 1
#define SERVO_2_PIN 7 // Chân điều khiển servo 2

#define NUM_MOTORS 4 // Số lượng động cơ
Const int motorPins[NUM_MOTORS][2] = { // Mảng chứa các chân điều khiển động cơ
  {8, 9},    // Motor 1
  {10, 11},  // Motor 2
  {12, 13},  // Motor 3
  {14, 15}   // Motor 4
};
// Khởi tạo đối tượng PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// Các kênh trên PCA9685 để điều khiển động cơ
int motorChannel1 = 0; // Điều khiển chiều quay
int motorChannel2 = 1; // Điều khiển tốc độ

// Khai báo các biến trạng thái
int analogTrai, analogPhai;
int nutAn;
int triggerL2;

Servo servo1; // servo1
Servo servo2; // servo2

void setup() {
  // Khởi tạo PS2 controller
  int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true); // Khởi tạo PS2 controller
  If (error == 0) { // Kiểm tra xem PS2 controller có kết nối được không
    Serial.println(“PS2 True!”);
  } else {
    Serial.println(“PS2 False!”);
  }
  // Khởi động giao tiếp I2C
  Wire.begin(); 
  
  // Khởi tạo PCA9685 với tần số PWM là 50Hz
  pca9685.begin();
  pca9685.setPWMFreq(50);

  // Khởi tạo các servo
  Servo1.attach(SERVO_1_PIN); // Khởi tạo servo 1
  Servo2.attach(SERVO_2_PIN); // Khởi tạo servo 2

  // Khởi tạo các chân điều khiển động cơ
  for (int i = 0; i < NUM_MOTORS; i++) { // Lặp qua từng động cơ
    pinMode(motorPins[i][0], OUTPUT);    // Chân điều khiển động cơ 1
    pinMode(motorPins[i][1], OUTPUT);    // Chân điều khiển động cơ 2
  }

  // Khởi tạo giao tiếp Serial
  Serial.begin(12600); 
}

void controlMotors(int analogValue, int motorIndex1, int motorIndex2) { // Hàm điều khiển động cơ
  if (analogValue > 128) { // Đi tiến
    // Rẽ phải
    digitalWrite(motorPins[motorIndex1][0], HIGH); 
    digitalWrite(motorPins[motorIndex1][1], LOW); 
    digitalWrite(motorPins[motorIndex2][1], LOW); // Motor 2 chậm lại hoặc dừng
    digitalWrite(motorPins[motorIndex2][0], LOW);
  } else if (analogValue < 128) { 
    // Rẽ trái
    digitalWrite(motorPins[motorIndex1][0], LOW); // Motor 1 chậm lại hoặc dừng
    digitalWrite(motorPins[motorIndex1][1], LOW);
    digitalWrite(motorPins[motorIndex2][1], HIGH);
    digitalWrite(motorPins[motorIndex2][0], LOW);
  } else {
    // Đi thẳng hoặc dừng
    digitalWrite(motorPins[motorIndex1][0], LOW); 
    digitalWrite(motorPins[motorIndex1][1], LOW);
    digitalWrite(motorPins[motorIndex2][1], LOW);
    digitalWrite(motorPins[motorIndex2][0], LOW);
  }
}

void setMotorDirection(bool forward) { // Hàm điều khiển chiều quay của động cơ
  if (forward) { // Quay theo chiều thuận
    pca9685.setPWM(motorChannel1, 0, 4095); // Quay theo chiều thuận
    pca9685.setPWM(motorChannel2, 0, 0); // Tắt chiều ngược lại
  } else { 
    pca9685.setPWM(motorChannel1, 0, 0); // Tắt chiều thuận
    pca9685.setPWM(motorChannel2, 0, 4095); // Quay theo chiều ngược lại
  }
}

void changeMotorSpeedGradually(int startSpeed, int endSpeed) { // Hàm thay đổi tốc độ động cơ
  int step = (startSpeed < endSpeed) ? 10 : -10; // Tăng hoặc giảm tốc

  for (int speed = startSpeed; (step > 0) ? (speed <= endSpeed) : (speed >= endSpeed); speed += step) { // Lặp qua từng tốc độ
    pca9685.setPWM(motorChannel2, 0, speed); // Thiết lập tốc độ cho chân điều khiển động cơ
    delay(20); // Điều chỉnh độ mượt mà của thay đổi tốc độ
  }
}
void loop() {
  Ps2x.read_gamepad(false, 0); // Đọc dữ liệu từ gamepad PS2

  // Đọc tín hiệu từ gamepad PS2
  analogTrai = ps2x.Analog(PSS_LX); // Lấy giá trị của cần analog trái (trục X)
  analogPhai = ps2x.Analog(PSS_RX); // Lấy giá trị của cần analog phải (trục X)
  nutAn = ps2x.Button(PSB_CROSS);   // Đọc tín hiệu từ nút X (Cross)
  triggerL2 = ps2x.Button(PSB_L2);  // Đọc tín hiệu từ Trigger L2
  PSB_PAD_UP = ps2x.Button(PSB_PAD_UP); // Đọc tín hiệu từ nút nhấn trên (Pad Up)
  PSB_PAD_DOWN = ps2x.Button(PSB_PAD_DOWN); // Đọc tín hiệu từ nút nhấn dưới (Pad Down)
  PSB_SQUARE = ps2x.Button(PSB_SQUARE); // Đọc tín hiệu từ nút nhấn vuông (Square)
  PSB_CIRCLE = ps2x.Button(PSB_CIRCLE); // Đọc tín hiệu từ nút nhấn tròn (Circle)
  PSB_TRIANGLE = ps2x.Button(PSB_TRIANGLE); // Đọc tín hiệu từ nút nhấn tam giác (Triangle)
  // Điều khiển motor dựa trên tín hiệu analog
  controlMotors(analogTrai, 0, 1);  // Điều khiển motor 1 và 2(tiến/lùi)
  controlMotors(analogPhai, 0, 1);  // Điều khiển motor 1 và 2(trái/phải)

int motor4Speed = 2765; // Giả sử tốc độ hiện tại của motor 4 là 2765 (Khoảng 60-70%speed)

  // Kiểm tra nếu tốc độ của motor 4 đạt giá trị lớn nhất
  if (motor4Speed >= 3071) { // Nếu motor 4 đạt trên 75% speed
      // Duy trì tốc độ tối đa
      pca9685.setPWM(motorChannel2, 3071, 4095); // Giữ motor 4 ở tốc độ tối đa
  } 
  // Kiểm tra nếu tốc độ của motor 4 thấp dưới 60%
  else if (motor4Speed < 650) { // Nếu motor 4 đạt dưới khoảng 15% speed 
      // Bật motor 3 quay ngược chiều kim đồng hồ
      digitalWrite(motorPins[2][0], LOW); // Tắt motor 3
      digitalWrite(motorPins[2][1], HIGH); // Bật motor 3

      // Tắt motor 4
      pca9685.setPWM(motorChannel2, 0, 0); // Tắt motor 4
  }
// Điều khiển động cơ quay theo chiều thuận
  setMotorDirection(true);
  changeMotorSpeedGradually(0, 4095); // Tăng tốc từ 0 lên 100%
  delay(2000); // Giữ tốc độ tối đa trong 2 giây

  // Giảm tốc động cơ về 0
  changeMotorSpeedGradually(4095, 0);
  delay(1000); // Dừng trong 1 giây
  // Điều khiển động cơ quay theo chiều ngược lại
  setMotorDirection(false);
  changeMotorSpeedGradually(0, 4095); // Tăng tốc từ 0 lên 100%
  delay(2000); // Giữ tốc độ tối đa trong 2 giây
  
  // Giảm tốc động cơ về 0
  changeMotorSpeedGradually(4095, 0);
  delay(1000); // Dừng trong 1 giây
  if (ps2x.Button(PSB_TRIANGLE)){ // Điều khiển motor 3 (quay chổi quét)
    digitalWrite(motorPins[2][0], HIGH); 
    digitalWrite(motorPins[2][1], LOW);
  } else 
      If (ps2x.Button(PSB_SQUARE)){ // Điều khiển motor 3 (dừng chổi quét)
          digitalWrite(motorPins[2][0], LOW);
           digitalWrite(motorPins[2][1], LOW);
         } else if (ps2x.Button(CIRCLE)) { // Điều khiển motor 3 (quay ngược chổi quét)
            digitalWrite(motorPins[2][0], LOW);
             digitalWrite(motorPins[2][1], HIGH);
           }
 // Điều khiển motor4 bắn bóng+xác định trạng thái của triggerL2
bool motorState = (triggerL2 == true) ? HIGH : LOW; // Nếu triggerL2 được ấn, gán giá trị HIGH cho motorState, ngược lại gán giá trị LOW cho motorState
 
// Cài đặt chân điều khiển động cơ dựa trên trạng thái triggerL2
digitalWrite(motorPins[3][0], motorState); 
digitalWrite(motorPins[3][1], LOW); 
    
  // Điều khiển servo khi nút bấm được nhấn
  if (ps2x.Button(PSB_PAD_UP)) { // Nếu nút mũi tên lên trên (Pad Up)
    Servo1.write(servo1 ? 90 : 0); 
    Servo2.write(servo2 ? 90 : 0);
  }  else 
        If (ps2x.Button(PSB_PAD_DOWN)){ // Nếu nút mũi tên xuống dưới (Pad Down)
          Servo1.write(0);
             Servo2.write(0);
}
  delay(100); // độ trễ để làm mượt hoạt động
}


