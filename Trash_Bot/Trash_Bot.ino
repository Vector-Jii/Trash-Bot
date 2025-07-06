#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>

const char *ssid = "Motor_Control";  
const char *password = "12345678";   

WiFiServer server(80);

// Define motor control pins
#define M1_M2_IN1 13
#define M1_M2_IN2 12
#define M3_M4_IN3 14
#define M3_M4_IN4 27
#define ENA 32
#define ENB 33

// Create PWM servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels on PCA9685
#define SERVO_CLAW_L 0
#define SERVO_CLAW_R 1
#define SERVO_ARM_1 2
#define SERVO_ARM_2 3
#define SERVO_TRASH_BIN 4

// Servo pulse lengths (adjust as needed)
#define SERVO_MIN_PULSE 150   // Minimum pulse length (0 degrees)
#define SERVO_MAX_PULSE 600   // Maximum pulse length (180 degrees)

void setup() {
    Serial.begin(115200);
    
    pinMode(M1_M2_IN1, OUTPUT);
    pinMode(M1_M2_IN2, OUTPUT);
    pinMode(M3_M4_IN3, OUTPUT);
    pinMode(M3_M4_IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

    WiFi.softAP(ssid, password);
    server.begin();
    Serial.println("AP Started. Connect to 'Motor_Control' and open 192.168.4.1");
    
    // Set initial speed (0-255)
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);

    // Set servos to initial position
    setServoAngle(SERVO_CLAW_L, 0);
    setServoAngle(SERVO_CLAW_R, 180);
    setServoAngle(SERVO_ARM_1, 90);
    setServoAngle(SERVO_ARM_2, 90);
    setServoAngle(SERVO_TRASH_BIN, 0);
}

void loop() {
    WiFiClient client = server.available();
    if (!client) return;

    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    // Motor Controls
    if (request.indexOf("/Forward") != -1) {
        digitalWrite(M1_M2_IN1, HIGH);
        digitalWrite(M1_M2_IN2, LOW);
        digitalWrite(M3_M4_IN3, HIGH);
        digitalWrite(M3_M4_IN4, LOW);
    }
    if (request.indexOf("/Backward") != -1) {
        digitalWrite(M1_M2_IN1, LOW);
        digitalWrite(M1_M2_IN2, HIGH);
        digitalWrite(M3_M4_IN3, LOW);
        digitalWrite(M3_M4_IN4, HIGH);
    }
    if (request.indexOf("/Left") != -1) {
        digitalWrite(M1_M2_IN1, LOW);
        digitalWrite(M1_M2_IN2, HIGH);
        digitalWrite(M3_M4_IN3, HIGH);
        digitalWrite(M3_M4_IN4, LOW);
    }
    if (request.indexOf("/Right") != -1) {
        digitalWrite(M1_M2_IN1, HIGH);
        digitalWrite(M1_M2_IN2, LOW);
        digitalWrite(M3_M4_IN3, LOW);
        digitalWrite(M3_M4_IN4, HIGH);
    }
    if (request.indexOf("/Stop") != -1) {
        stopMotors();
    }

    // Servo Controls
    if (request.indexOf("/ClawOpen") != -1) {
        setServoAngle(SERVO_CLAW_L, 0);
        setServoAngle(SERVO_CLAW_R, 180);
    }
    if (request.indexOf("/ClawClose") != -1) {
        setServoAngle(SERVO_CLAW_L, 100);
        setServoAngle(SERVO_CLAW_R, 80);
    }
    if (request.indexOf("/Pick") != -1) {
    // Extend arm downward
    setServoAngle(SERVO_ARM_1, 180);  // vertically up
    setServoAngle(SERVO_ARM_2, 0);
    delay(1000);

    // Close claw to grab trash
    setServoAngle(SERVO_CLAW_L, 110);
    setServoAngle(SERVO_CLAW_R, 70);
    delay(1000);

    // Retract arm
    setServoAngle(SERVO_ARM_1, 160);
    setServoAngle(SERVO_ARM_2, 20);
    }
    if (request.indexOf("/Extend") != -1) {
        setServoAngle(SERVO_ARM_1, 160);
        setServoAngle(SERVO_ARM_2, 20);
    }
    if (request.indexOf("/Retract") != -1) {
        setServoAngle(SERVO_ARM_1, 30);
        setServoAngle(SERVO_ARM_2, 150);
    }
    if (request.indexOf("/TrashRotate") != -1) {
        setServoAngle(SERVO_TRASH_BIN, 180);
        delay(1000);
        setServoAngle(SERVO_TRASH_BIN, 0);
    }

    // Webpage Response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();
    client.println("<html><body>");
    client.println("<h2>ESP32 Trash Bot Control</h2>");
    
    // Motor Buttons
    client.println("<h3>Movement</h3>");
    client.println("<a href='/Forward'><button>Forward</button></a>");
    client.println("<a href='/Backward'><button>Backward</button></a>");
    client.println("<a href='/Left'><button>Left</button></a>");
    client.println("<a href='/Right'><button>Right</button></a>");
    client.println("<a href='/Stop'><button style='background:red;'>STOP</button></a>");

    // Servo Buttons
    client.println("<h3>Claw Control</h3>");
    client.println("<a href='/ClawOpen'><button>Open Claw</button></a>");
    client.println("<a href='/ClawClose'><button>Close Claw</button></a>");
    
    client.println("<h3>Arm Control</h3>");
    client.println("<a href='/Pick'><button style='background:green;'>Pick</button></a>");
    client.println("<a href='/Extend'><button>Extend Arm</button></a>");
    client.println("<a href='/Retract'><button>Retract Arm</button></a>");

    client.println("<h3>Trash Bin</h3>");
    client.println("<a href='/TrashRotate'><button>Rotate Bin</button></a>");

    client.println("</body></html>");
    client.println();
}

void stopMotors() {
    digitalWrite(M1_M2_IN1, LOW);
    digitalWrite(M1_M2_IN2, LOW);
    digitalWrite(M3_M4_IN3, LOW);
    digitalWrite(M3_M4_IN4, LOW);
}

// Helper function to set servo angle using PCA9685
void setServoAngle(uint8_t servoNum, uint8_t angle) {
    // Map angle (0-180) to pulse length
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(servoNum, 0, pulse);
}