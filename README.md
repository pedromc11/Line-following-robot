# 🤖 Line-Following Robot (PlatformIO)

---

## 🌟 Overview

Welcome to the repository for my **Line-Following Robot**! This project, developed using **PlatformIO** for seamless development and deployment, showcases a robot designed to autonomously navigate a path by following a black line. Beyond simple line-following, this robot is equipped with **obstacle avoidance** capabilities and provides **interactive feedback** through LEDs and sounds, making it a versatile and engaging platform for robotics enthusiasts and learners.

---

## ✨ Features

This robot isn't just about following lines; it's packed with smart functionalities:

* **Precise Line Following:** Utilizes **infrared (IR) sensors** to accurately detect the black line. The robot intelligently adjusts its motor speeds to stay on track, even with curves and turns.
* **Intelligent Obstacle Avoidance:** An integrated **ultrasonic sensor** constantly monitors the environment. If an obstacle is detected too close, the robot performs an evasive maneuver, ensuring it doesn't collide.
* **Interactive Control:** A dedicated **push button** offers a simple way to trigger a specific action – a quick 180-degree turn – perfect for resetting its path or initiating a special sequence.
* **Visual Feedback:** **Red and green LEDs** provide clear visual cues about the robot's status. Green indicates normal operation, while red lights up during the special turn.
* **Auditory Alerts:** A **buzzer** provides immediate auditory feedback, playing a short melody when an obstacle is detected, adding another layer of interaction and awareness.
* **Ambient Light Adaptation:** An **analog light sensor** helps the robot adapt to different lighting conditions, ensuring its line-following logic remains robust whether in bright or dim environments.
* **PlatformIO Development:** Built with PlatformIO, this project benefits from a streamlined development workflow, enhanced library management, and cross-platform compatibility.

---

## ⚙️ Hardware Components

To build this robot, you'll need the following key components:

* **Microcontroller Board:** An Arduino-compatible board (e.g., ESP32, ESP8266, Arduino Uno, etc., compatible with PlatformIO).
* **Adafruit PWM Servo Driver (PCA9685):** Essential for controlling the continuous rotation servos.
* **Continuous Rotation Servos (x2):** For the robot's movement (left and right wheels).
* **Standard Servo (e.g., SG90/MG90S):** Used for specific maneuvers, likely for the ultrasonic sensor's sweep or another mechanism.
* **Infrared (IR) Line Following Sensors (x2):** To detect the line on the ground.
* **Ultrasonic Distance Sensor (HC-SR04):** For obstacle detection.
* **Push Button:** For user interaction.
* **LEDs (Red & Green):** For visual feedback.
* **Buzzer:** For auditory feedback.
* **Photoresistor (LDR):** For ambient light sensing.
* **Jumper Wires, Breadboard (optional), Robot Chassis, Wheels.**

---

## 🛠️ Software Setup (PlatformIO)

This project is designed to be compiled and uploaded using **PlatformIO**, a professional embedded development environment.

1.  **Install PlatformIO Core:** If you haven't already, install PlatformIO. The easiest way is via the VS Code extension.
    ```bash
    pip install -U platformio
    ```
2.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git](https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git)
    cd YOUR_REPO_NAME
    ```
3.  **Open in PlatformIO:** Open the project folder in your IDE (e.g., VS Code with PlatformIO extension).
4.  **Install Dependencies:** PlatformIO will automatically detect and prompt you to install the necessary libraries (`Adafruit PWM Servo Driver Library` and `Wire`). If not, you can add them manually via the PlatformIO Library Manager.
5.  **Configure `platformio.ini`:** Ensure your `platformio.ini` file is configured for your specific microcontroller board. An example configuration might look like:
    ```ini
    [env:esp32dev] # Or your specific board, e.g., uno, nodemcuv2, etc.
    platform = espressif32 # Or atmelavr, espressif8266, etc.
    board = esp32dev
    framework = arduino
    lib_deps =
        adafruit/Adafruit PWM Servo Driver Library@^X.X.X
        wire
    ```
    *(Replace `X.X.X` with the latest stable version of the Adafruit PWM Servo Driver Library).*
6.  **Upload Code:** Connect your microcontroller board and click the "Upload" button in PlatformIO to compile and flash the code.

---

## 🚀 How It Works

The robot's behavior is orchestrated in the `loop()` function, constantly reading sensor data and making decisions:

1.  **Sensor Readings:**
    * **IR Sensors:** `valor_IR_left` and `valor_IR_right` detect the line. A `negro` (black) value indicates the sensor is over the line, while `blanco` (white) means it's off the line.
    * **Ultrasonic Sensor:** The `cm` variable calculates the distance to the nearest object.
    * **Light Sensor:** `light` reads the ambient light intensity.
    * **Push Button:** `button_value` checks if the button is pressed.

2.  **Obstacle Avoidance Logic:**
    * If `cm` (distance) is less than 9 cm, the robot activates `tocar()` (plays a melody) and performs a pre-programmed sequence of movements using the servos to navigate around the obstacle.

3.  **Button Press Action:**
    * If the `button_value` is HIGH (pressed), the **red LED** turns on, the green LED turns off, and the robot executes a specific turn for 3.8 seconds.
    * Otherwise, the **green LED** is on, indicating normal line-following mode.

4.  **Line Following Logic:**
    * The core line-following is based on a series of `if-else if` statements, evaluating the combined readings from the **IR sensors** and the **light sensor (`light`)**. This allows the robot to react differently based on whether it's perfectly on the line, slightly off to the left or right, or even if the ambient lighting changes drastically.
    * **`SERVOMIN`**, **`SERVOSTOP`**, and **`SERVOMAX`** define the PWM values for forward, stop, and reverse motion of the continuous rotation servos. The code manipulates `pwm.setPWM(servo_left, 0, VALUE)` and `pwm.setPWM(servo_right, 0, VALUE)` to control the wheels.
    * The `light` sensor helps differentiate scenarios, for instance, if both IR sensors are `negro` but the ambient light is very low (indicating perhaps a very wide dark line or a different environment).

5.  **`tocar()` Function:**
    * This separate function is called when an obstacle is detected. It uses `tone()` to play a sequence of musical notes on the buzzer (`Buzzpin`), providing an audible warning.

---

## 🤝 Contribution

Feel free to **fork** this repository, experiment with the code, and suggest improvements! Whether it's enhancing the line-following algorithm, refining obstacle avoidance maneuvers, or adding new features, all contributions are welcome.

---

## 📄 License

This project is open-source and available under the [MIT License](LICENSE).

---

Feel free to reach out if you have any questions or ideas!
