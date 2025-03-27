#include <Servo.h>
#include <NewPing.h>

// Định nghĩa chân cho motor
int enL = 7;
int inL1 = 8;
int inL2 = 9;

int inR1 = 10;
int inR2 = 11;
int enR = 12;
int enLA = 2;
int enLB = 3;
int enRA = 18;
int enRB = 19;
const float GRIP_DISTANCE = 5.0;
bool hasGrabbed = false;

// Định nghĩa chân cho HC-SR04
// const int trigPin = 22;
// const int echoPin = 24;
// Định nghĩa chân cho HC-SR04
const int trigPin = 22;
const int echoPin = 24;
const int maxDistance = 200; // Khoảng cách tối đa (cm)

NewPing sonar(trigPin, echoPin, maxDistance); // Tạo đối tượng NewPing
// Định nghĩa cho servo
Servo gripper;
const int servoPin = 4;

// Biến lưu tốc độ cơ bản
const int baseSpeed = 80;

// Cài đặt ban đầu
void setup() {
  Serial.begin(9600);  // Khởi động Serial để giao tiếp với Raspberry Pi

  // Cài đặt chân motor
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  // Cài đặt chân HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Cài đặt servo
  gripper.attach(servoPin);
  gripper.write(0);  // Vị trí ban đầu

  Serial.println("Arduino Ready");
}
// Hàm đo khoảng cách từ HC-SR04 sử dụng NewPing
float getDistance() {
  unsigned int distance = sonar.ping_cm(); // Lấy khoảng cách (cm)
  return distance == 0 ? maxDistance : distance; // Nếu không đo được, trả về maxDistance
}


// Vòng lặp chính
void loop() {
  // float distance = getDistance();
  // Serial.println(distance);
  // Serial.println("Haha");
    // Đo khoảng cách liên tục
  // Nếu đã gắp xong, không thực hiện thêm hành động nào
  if (hasGrabbed) {
    stopMotors();
    delay(1000); // Tránh treo vòng lặp
    return;
  }

  // Đo khoảng cách liên tục
  float distance = getDistance();

  // Nếu khoảng cách nhỏ hơn GRIP_DISTANCE và chưa gắp
  if (distance > 0 && distance <= GRIP_DISTANCE && !hasGrabbed) {
    Serial.println("Object detected within grip range.");
    grabObject();        // Gọi hàm gắp vật
    stopMotors();        // Dừng robot
    hasGrabbed = true;   // Đánh dấu đã gắp
    Serial.println("GRABBED"); // Gửi trạng thái về Python
    return;
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.println("Command received: " + command);

    if (command == "LEFT") {
      Serial.println("Turning Left");
      turnLeft();
      // delay(300);
    } else if (command == "RIGHT") {
      Serial.println("Turning Right");
      turnRight();
      // delay(300);
    } else if (command == "CENTER") {
      Serial.println("Moving Forward");
      moveForward();
    } else if (command == "GRAB") {
      Serial.println("Grab");
      // gripper.write(0);  
      // delay(500);
      gripper.write(80);  // Close gripper
      delay(1000);
      gripper.write(0); // Open gripper after gripping (if required)
    } else if (command == "STOP") {
      // Serial.println("Stopping Motors");
      stopMotors();
    } else if (command == "SLOW FORWARD") {
      // Serial.println("Slow down a bit");
      moveForwardbit();
      // delay(300);
    } else if (command == "DISTANCE") {
      float distance = getDistance();
      Serial.println(distance);  // Gửi khoảng cách về Raspberry Pi
    }
  }
}

// Hàm điều khiển motor
void moveForward() {
  analogWrite(enL, baseSpeed);
  analogWrite(enR, baseSpeed);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}
void moveForwardbit() {
  analogWrite(enL, 70);
  analogWrite(enR, 70);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnLeft() {
  analogWrite(enL, 100);
  analogWrite(enR, baseSpeed);

  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnRight() {
  analogWrite(enL, baseSpeed);
  analogWrite(enR, 100);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void stopMotors() {
  analogWrite(enL, 0);
  analogWrite(enR, 0);

  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

// Hàm gắp vật thể
void grabObject() {
  Serial.println("Grabbing object...");
  gripper.write(80);  // Đóng gắp
  delay(1000);        // Chờ servo thực hiện
  gripper.write(0);   // Mở lại nếu cần
}
