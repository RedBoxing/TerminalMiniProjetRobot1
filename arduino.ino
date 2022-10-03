#include <Servo.h>

#define echoPin 2
#define trigPin 3

#define MOTOR_A_DIRECTION 12
#define MOTOR_B_DIRECTION 13

#define MOTOR_A_PWM 3
#define MOTOR_B_PWM 11

#define ROBOT_SPEED 1

Servo sensor_servo;

struct SensorInfo {
    long duration;
    int distance;
}

SensorInfo getSensorInfos() {
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor

    return {
        duration,
        distance
    }
}

void rotateSensor(int angle) {
    sensor_servo.write(angle);
}

void setRightSpeed(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
        digitalWrite(MOTOR_B_DIRECTION, LOW);
    }
    else
    {
        digitalWrite(MOTOR_B_DIRECTION, HIGH);
    }

    analogWrite(MOTOR_B_PWM, speed);
}

void setLeftSpeed(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
        digitalWrite(MOTOR_A_DIRECTION, LOW);
    }
    else
    {
        digitalWrite(MOTOR_A_DIRECTION, HIGH);
    }

    analogWrite(MOTOR_A_PWM, speed);
}

void stop()
{
    setLeftSpeed(0);
    setRightSpeed(0);
}

void move_forward()
{
    setLeftSpeed(100);
    setRightSpeed(100);
}

void move_backward()
{
    setLeftSpeed(-100);
    setRightSpeed(-100);
}

void move_left()
{
    setLeftSpeed(-100);
    setRightSpeed(100);
}

void move_right()
{
    setLeftSpeed(100);
    setRightSpeed(-100);
}

void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
}

void loop() {
    move_forward();
    delay(1000);
    stop();

    SensorInfo initialInfos = getSensorInfos();
    rotateSensor(-90);
    
    move_forward();
    SensorInfo infos = getSensorInfos();
    int neededTime = (initialInfos.distance / ROBOT_SPEED) / 1000;
    int elapsedTime = 0;

    bool needToTurn = false;

    while(elapsedTime < neededTime) {
        infos = getSensorInfos();
        elapsedTime = millis();
        if(infos.distance < 2) {
            needToTurn = true;
            break;
        }
    }

    if(needToTurn) {
        stop();
        move_left();
        delay(1000);
        stop();
    }
}