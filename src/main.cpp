#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- Servo and Motor Constants ---
// Create an instance of the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Continuous Rotation Servo (Wheels) PWM values
const int SERVO_MIN = 320;   // Full speed reverse
const int SERVO_STOP = 380;  // Stop
const int SERVO_MAX = 440;   // Full speed forward

// Servo channels for wheels
const int SERVO_LEFT_CHANNEL = 0;
const int SERVO_RIGHT_CHANNEL = 1;

// Standard Servo (e.g., for ultrasonic sensor sweep) PWM values
const int SERVO_45_DEG = 300;
const int SERVO_90_DEG = 420;
const int SERVO_135_DEG = 540;
const int SERVO_ULTRASONIC_CHANNEL = 2; // Assuming this is servo_180 from original code

// --- Sensor Pin Definitions ---
const int IR_LEFT_PIN = 2;
const int IR_RIGHT_PIN = 3;
const int PING_PIN = 5; // Ultrasonic sensor trig/echo pin
const int ANALOG_LIGHT_PIN = A0; // Analog pin for light sensor

// --- LED Pin Definitions ---
const int RED_LED_PIN = 12;
const int GREEN_LED_PIN = 9;

// --- Button Pin Definition ---
const int BUTTON_PIN = 11;

// --- Music (Buzzer) Constants ---
const int BUZZER_PIN = 13;
const int NOTE_DO = 523;
const int NOTE_RE = 587;
const int NOTE_MI = 659;
const int NOTE_FA = 698;
const int NOTE_SOL = 784;
const int NOTE_LA = 880;
const int NOTE_SI = 988;
const int NOTE_DOS = 1047;

// --- Thresholds for Line Sensors and Light Sensor ---
// Using an enum for clarity on IR sensor states
enum IRSensorState {
  IR_BLACK = 0,
  IR_WHITE = 1
};

// Light sensor thresholds
const int LIGHT_NORMAL_MIN = 400;
const int LIGHT_NORMAL_MAX = 600;
const int LIGHT_LEFT_MAX = 400; // Less than or equal to 400
const int LIGHT_RIGHT_MIN = 600; // Greater than or equal to 600

// --- Global Variables ---
unsigned long circuit_time_ms = 0; // Using a more descriptive name

// --- Function Prototypes (Good practice for larger projects) ---
void playObstacleMelody(); // Renamed from 'tocar' for better readability and context
long readUltrasonicDistanceCm();
void setMotorSpeeds(int leftSpeed, int rightSpeed);


// --- Setup Function ---
void setup() {
  pwm.begin();
  pwm.setPWMFreq(60); // Set PWM frequency to 60Hz

  // Initialize digital input/output pins
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  // PING_PIN will be set as OUTPUT/INPUT dynamically in readUltrasonicDistanceCm()
  pinMode(BUZZER_PIN, OUTPUT); // Ensure buzzer pin is set as output

  Serial.begin(9600); // Initialize serial communication for debugging
  Serial.println("Robot initialized. Waiting for commands...");
}

// --- Main Loop Function ---
void loop() {
  // Read sensor values
  int ir_left_value = digitalRead(IR_LEFT_PIN);
  int ir_right_value = digitalRead(IR_RIGHT_PIN);
  int ambient_light_value = analogRead(ANALOG_LIGHT_PIN);
  int button_state = digitalRead(BUTTON_PIN);
  int distance_cm = readUltrasonicDistanceCm(); // Call function to get ultrasonic distance

  // --- Obstacle Avoidance Logic ---
  if (distance_cm < 9) {
    playObstacleMelody();
    // Execute a series of maneuvers to avoid the obstacle
    setMotorSpeeds(SERVO_STOP, SERVO_MAX); // Turn right
    pwm.setPWM(SERVO_ULTRASONIC_CHANNEL, 0, SERVO_90_DEG); // Adjust ultrasonic servo
    delay(1000);

    setMotorSpeeds(SERVO_MIN, SERVO_MAX); // Move back/adjust
    pwm.setPWM(SERVO_ULTRASONIC_CHANNEL, 0, SERVO_135_DEG); // Adjust ultrasonic servo
    delay(1000);

    setMotorSpeeds(SERVO_MIN, SERVO_STOP); // Turn left
    pwm.setPWM(SERVO_ULTRASONIC_CHANNEL, 0, SERVO_90_DEG); // Adjust ultrasonic servo
    delay(1000);

    setMotorSpeeds(SERVO_MIN, SERVO_MAX); // Move forward/adjust
    pwm.setPWM(SERVO_ULTRASONIC_CHANNEL, 0, SERVO_90_DEG); // Adjust ultrasonic servo
    delay(1000);
  }

  // --- Button Press Logic ---
  if (button_state == HIGH) {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH); // Red LED on when button is pressed
    setMotorSpeeds(SERVO_STOP, 410); // Specific turn for 180 degrees
    delay(3800);
    digitalWrite(RED_LED_PIN, LOW); // Turn off red LED after maneuver
  } else {
    // Green LED on when button is not pressed
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  }

  // --- Line Following Logic ---
  // This section uses a series of if-else if statements to determine robot movement
  // based on IR sensor readings and ambient light.

  // Scenario 1: Normal Light (400-600)
  if (ambient_light_value >= LIGHT_NORMAL_MIN && ambient_light_value <= LIGHT_NORMAL_MAX) {
    if (ir_right_value == IR_BLACK && ir_left_value == IR_BLACK) {
      setMotorSpeeds(SERVO_MIN, SERVO_MAX); // Both on black: Go straight
    } else if (ir_right_value == IR_WHITE && ir_left_value == IR_BLACK) {
      setMotorSpeeds(SERVO_STOP, SERVO_MAX); // Right off, left on: Turn right
    } else if (ir_right_value == IR_BLACK && ir_left_value == IR_WHITE) {
      setMotorSpeeds(SERVO_MIN, SERVO_STOP); // Left off, right on: Turn left
    } else if (ir_right_value == IR_WHITE && ir_left_value == IR_WHITE) {
      setMotorSpeeds(SERVO_MIN, SERVO_MAX); // Both off: Keep going straight (adjust as needed for lost line)
    }
  }
  // Scenario 2: Low Light (<= 400) - Robot might behave differently in dark
  else if (ambient_light_value >= 0 && ambient_light_value <= LIGHT_LEFT_MAX) {
    if (ir_right_value == IR_BLACK && ir_left_value == IR_BLACK) {
      setMotorSpeeds(SERVO_STOP, SERVO_MAX); // Both on black: Turn right (might be on a wide line or edge)
    } else if (ir_right_value == IR_WHITE && ir_left_value == IR_BLACK) {
      setMotorSpeeds(SERVO_STOP, SERVO_MAX); // Right off, left on: Turn right
    } else if (ir_right_value == IR_BLACK && ir_left_value == IR_WHITE) {
      setMotorSpeeds(SERVO_MIN, SERVO_MAX); // Left off, right on: Go straight (might be on a curve)
    } else if (ir_right_value == IR_WHITE && ir_left_value == IR_WHITE) {
      setMotorSpeeds(SERVO_MIN, SERVO_STOP); // Both off: Turn left (might be lost or trying to find line)
    }
  }
  // Scenario 3: High Light (>= 600) - Robot might behave differently in bright light
  else if (ambient_light_value >= LIGHT_RIGHT_MIN) {
    if (ir_right_value == IR_BLACK && ir_left_value == IR_BLACK) {
      setMotorSpeeds(SERVO_MIN, SERVO_STOP); // Both on black: Turn left (might be on a wide line or edge)
    } else if (ir_right_value == IR_WHITE && ir_left_value == IR_BLACK) {
      setMotorSpeeds(SERVO_MIN, SERVO_MAX); // Right off, left on: Go straight (might be on a curve)
    } else if (ir_right_value == IR_BLACK && ir_left_value == IR_WHITE) {
      setMotorSpeeds(SERVO_MIN, SERVO_STOP); // Left off, right on: Turn left
    } else if (ir_right_value == IR_WHITE && ir_left_value == IR_WHITE) {
      setMotorSpeeds(SERVO_STOP, SERVO_MAX); // Both off: Turn right (might be lost or trying to find line)
    }
  }

  // --- Debugging Output ---
  circuit_time_ms = millis();
  Serial.println(circuit_time_ms);
}

// --- Function to Play Obstacle Melody ---
void playObstacleMelody() {
  tone(BUZZER_PIN, NOTE_FA);
  delay(300);
  tone(BUZZER_PIN, NOTE_DO);
  delay(300);
  tone(BUZZER_PIN, NOTE_DO);
  delay(300);
  tone(BUZZER_PIN, NOTE_FA);
  delay(300);
  tone(BUZZER_PIN, NOTE_SOL);
  delay(300);
  tone(BUZZER_PIN, NOTE_LA);
  delay(300);
  tone(BUZZER_PIN, NOTE_SOL);
  delay(300);
  tone(BUZZER_PIN, NOTE_DO);
  delay(300);
  tone(BUZZER_PIN, NOTE_FA);
  delay(300);
  tone(BUZZER_PIN, NOTE_SOL);
  delay(300);
  tone(BUZZER_PIN, NOTE_LA);
  delay(300);
  tone(BUZZER_PIN, NOTE_SI);
  delay(300);
  tone(BUZZER_PIN, NOTE_SOL);
  delay(300);
  tone(BUZZER_PIN, NOTE_LA);
  delay(300);
  tone(BUZZER_PIN, NOTE_SOL);
  delay(300);
  tone(BUZZER_PIN, NOTE_FA);
  delay(300);
  tone(BUZZER_PIN, NOTE_SOL);
  delay(300);
  tone(BUZZER_PIN, NOTE_LA);
  delay(300);
  tone(BUZZER_PIN, NOTE_MI);
  delay(300);
  tone(BUZZER_PIN, NOTE_LA);
  delay(300);
  tone(BUZZER_PIN, NOTE_RE);
  delay(300);
  tone(BUZZER_PIN, NOTE_FA);
  delay(300);
  noTone(BUZZER_PIN);
}

// --- Function to Read Ultrasonic Distance ---
long readUltrasonicDistanceCm() {
  // Clears the trigPin
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PING_PIN, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(PING_PIN, HIGH);

  // Calculate the distance (speed of sound ~ 343 meters/second or 0.0343 cm/microsecond)
  // Divide by 2 because the sound travels to the object and back.
  long cm = duration / 29 / 2;
  return cm;
}

// --- Function to Set Motor Speeds ---
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  pwm.setPWM(SERVO_LEFT_CHANNEL, 0, leftSpeed);
  pwm.setPWM(SERVO_RIGHT_CHANNEL, 0, rightSpeed);
}