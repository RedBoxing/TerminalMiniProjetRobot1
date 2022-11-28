#include <Servo.h>
#include <HCSR04.h>

#define MOTOR1_PWM 3
#define MOTOR2_PWM 11
#define MOTOR3_PWM 5
#define MOTOR4_PWM 6

#define MOTOR1_DIRECTION 4
#define MOTOR2_DIRECTION 12
#define MOTOR3_DIRECTION 8 
#define MOTOR4_DIRECTION 7

#define BT_TXD 10
#define BT_RXD 11

#define ECHO_PIN 2
#define TRIG_PIN 9

#define SENSOR_MOTOR 13

#define ROBOT_SPEED 1

Servo sensor_servo;
UltraSonicDistanceSensor distanceSensor(TRIG_PIN, ECHO_PIN);

struct SensorInfo {
    long duration;
    int distance;
};

SensorInfo getSensorInfos() {
   /* // Clears the trigPin condition
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH);
    // Calculating the distance
    int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
*/
  int distance = distanceSensor.measureDistanceCm();
  long duration = 0;

    return {
        duration,
        distance
    };
}

void rotateSensor(int angle) {
    sensor_servo.write(angle);
}

void setRightSpeed(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
        digitalWrite(MOTOR2_DIRECTION, LOW);
    }
    else
    {
        digitalWrite(MOTOR2_DIRECTION, HIGH);
    }

    analogWrite(MOTOR2_PWM, speed);
}

void setLeftSpeed(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
        digitalWrite(MOTOR1_DIRECTION, LOW);
    }
    else
    {
        digitalWrite(MOTOR1_DIRECTION, HIGH);
    }

    analogWrite(MOTOR1_PWM, speed);
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

    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR1_DIRECTION, OUTPUT);
    pinMode(MOTOR2_DIRECTION, OUTPUT);

    sensor_servo.attach(SENSOR_MOTOR);

    Serial.begin(9600);
}

void loop() {  
    SensorInfo initialInfos = getSensorInfos();
    rotateSensor(-90);
    
    move_forward();
    SensorInfo infos = getSensorInfos();
    Serial.print("Distance: ");
    Serial.println(initialInfos.distance);
    unsigned long neededTime = (initialInfos.distance) * 1000;

    Serial.print("Needed time: ");
    Serial.println(neededTime);
    unsigned long initialTime = millis();
    unsigned long elapsedTime = 0;

    Serial.print("Initial time: ");
    Serial.println(initialTime);

    bool needToTurn = false;

    while(elapsedTime < neededTime) {
        infos = getSensorInfos();
        elapsedTime = millis() - initialTime;
        Serial.print("Elapsed time: ");
        Serial.println(elapsedTime);
        if(infos.distance <= 2) {
            needToTurn = true;
            break;
        }
    }

    if(needToTurn) {
        stop();
        move_left();
        delay(500);
        stop();
    } else {
        rotateSensor(180);
        delay(500);
        stop();

        if(getSensorInfos().distance > 2) {
            move_right();
            delay(1000);
            stop();
        }
    }
}