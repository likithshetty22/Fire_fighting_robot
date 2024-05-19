# Fire-Fighting Robot

This project is an Arduino-based fire-fighting robot. It uses flame sensors to detect fire, moves towards the fire, and extinguishes it by spraying water. The robot is equipped with motors for movement, a water pump for extinguishing the fire, and a servo motor to direct the water spray.

## Components

- Arduino Uno (or compatible board)
- 3 Flame Sensors
- 2 DC Motors with Motor Driver
- Water Pump
- Servo Motor
- Connecting wires and power supply

## Pin Configuration

| Component            | Arduino Pin |
| -------------------- | ----------- |
| Left Flame Sensor    | 9           |
| Right Flame Sensor   | 10          |
| Forward Flame Sensor | 8           |
| Left Motor 1         | 2           |
| Left Motor 2         | 3           |
| Right Motor 1        | 4           |
| Right Motor 2        | 5           |
| Water Pump           | 6           |
| Servo Motor          | 7           |

## Code Overview

The code is divided into two main sections: `setup()` and `loop()`.

### Setup

- **Initialize Pins**: Configures the sensor pins as inputs, motor driver pins as outputs, and the water pump pin as an output.
- **Servo Motor**: Attaches the servo motor to the specified pin and sets its initial position.

### Loop

- **Fire Detection**: Continuously checks the flame sensors to detect fire.
- **Movement**:
  - If a fire is detected by the left sensor, the robot turns left.
  - If detected by the right sensor, it turns right.
  - If detected by the forward sensor, it moves forward.
  - If no fire is detected, the robot stops.
- **Water Pump Activation**:
  - When the robot is close to the fire, it turns on the water pump and sprays water.
  - The servo motor is used to direct the water spray.
  - When no fire is detected, it turns off the water pump and resets the servo motor.

## Usage

1. **Hardware Setup**:
   - Connect the flame sensors, motors, water pump, and servo motor to the Arduino as per the pin configuration table.
   - Ensure all components are powered appropriately.

2. **Upload Code**:
   - Open the Arduino IDE.
   - Copy the provided code into a new sketch.
   - Upload the sketch to your Arduino board.

3. **Run the Robot**:
   - Place the robot in an environment where it can detect and move towards fire.
   - Ensure there is a safe, controlled fire source for testing.
   - The robot will autonomously detect the fire, move towards it, and attempt to extinguish it.

## Code

```cpp
#include <Wire.h>
#include <Servo.h>

// Define the pins for the flame sensors, motors, and servo
const int leftSensor = 9;
const int rightSensor = 10;
const int forwardSensor = 8;
const int leftMotor1 = 2;
const int leftMotor2 = 3;
const int rightMotor1 = 4;
const int rightMotor2 = 5;
const int waterPump = 6;
const int servoPin = 7; // Adjust the pin number if necessary

// Create a servo object
Servo myservo;

// Declare a boolean variable to track water pump status and fire status
bool isWaterPumpOn = false;
bool fire = false;

// Initialize the flame sensors, motors, and servo
void setup()
{
    pinMode(leftSensor, INPUT);
    pinMode(rightSensor, INPUT);
    pinMode(forwardSensor, INPUT);
    pinMode(leftMotor1, OUTPUT);
    pinMode(leftMotor2, OUTPUT);
    pinMode(rightMotor1, OUTPUT);
    pinMode(rightMotor2, OUTPUT);
    pinMode(waterPump, OUTPUT); // Set the water pump pin as an output

    // Attach the servo to the pin
    myservo.attach(servoPin);

    // Set the initial position of the servo
    myservo.write(0);
}

// Loop forever
void loop()
{
    // Check if there is fire
    if (digitalRead(leftSensor) == HIGH || digitalRead(rightSensor) == HIGH || digitalRead(forwardSensor) == HIGH)
    {
        fire = true;
    }
    else
    {
        fire = false;
    }

    // If there is fire, move the robot towards it
    if (fire)
    {
        if (digitalRead(leftSensor) == HIGH)
        {
            // Turn left
            digitalWrite(leftMotor1, HIGH);
            digitalWrite(leftMotor2, LOW);
            digitalWrite(rightMotor1, LOW);
            digitalWrite(rightMotor2, HIGH);
        }
        else if (digitalRead(rightSensor) == HIGH)
        {
            // Turn right
            digitalWrite(leftMotor1, LOW);
            digitalWrite(leftMotor2, HIGH);
            digitalWrite(rightMotor1, HIGH);
            digitalWrite(rightMotor2, LOW);
        }
        else if (digitalRead(forwardSensor) == HIGH)
        {
            // Move forward
            digitalWrite(leftMotor1, HIGH);
            digitalWrite(leftMotor2, LOW);
            digitalWrite(rightMotor1, HIGH);
            digitalWrite(rightMotor2, LOW);
        }
    }
    else
    {
        // No fire detected, stop the motors
        digitalWrite(leftMotor1, LOW);
        digitalWrite(leftMotor2, LOW);
        digitalWrite(rightMotor1, LOW);
        digitalWrite(rightMotor2, LOW);
    }

    // If the robot is close to the fire, turn on the water pump and spray water
    if (fire && (digitalRead(leftSensor) == HIGH || digitalRead(rightSensor) == HIGH || digitalRead(forwardSensor) == HIGH))
    {
        digitalWrite(waterPump, HIGH);
        isWaterPumpOn = true;

        // Rotate the servo to spray water at the sight where the fire is detected
        myservo.write(120);
        delay(500); // Delay for spraying water
    }
    else
    {
        digitalWrite(waterPump, LOW);
        isWaterPumpOn = false;

        // Set the servo back to 0 degrees when the fire is extinguished
        myservo.write(0);
    }
}
