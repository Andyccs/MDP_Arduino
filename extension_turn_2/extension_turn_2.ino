#include <PololuWheelEncoders.h>
#include <DualVNH5019MotorShield.h>
#include <Average.h>
#include <PID_v1.h>

PololuWheelEncoders we;
DualVNH5019MotorShield md;

//this value is used to decrease the left motor speed
//it is used when turn left and right
//do not change the value
const double leftMotorAdjust = 0.975;

//number of revolution need to turn left and right
//do not change the value
const int perRevolutionCount = 1097;

//calculate for turn1Count
//do not change these values
const int wheelDiameter = 6;
const double robotDiameter = 17.0; 
const double turn90Count = (((((PI/2.0) * (robotDiameter/2.0)) / (PI*wheelDiameter)) * perRevolutionCount));
const double turn1Count = turn90Count/90;

//this value can be used tmo increase or decrease 
//the rpm when turn right
//change to higher value for more rotation
//change to lower value for less rotation
const double errorRight = 1.0225;

//this value can be used to increase or decrease 
//the rpm when turn left
//change to higher value for more rotation
//change to lower value for less rotation
const double errorLeft = 0.9900;

//Determines how often the PID algorithm evaluates. The default is 200mS.
//do not change the value
const int pidLoopTime = 100;  

//For debugging purpose
//set it to true to debug
//set it to false for production
const bool DEBUG = false;

//pin number for motor
//do not change
const int motor1_a = 4;
const int motor1_b = 2;
const int motor2_a = 8;
const int motor2_b = 7;

//Serial Read String Function
char Comp(){
    if(Serial.available()>0){
        return Serial.read();
    }else{
        return '\0';
    }
}

void setup()
{
    md.init();  
    we.init(11,13,3,5);
  
    Serial.begin(9600);
}

void loop(){
    char command = Comp();
    if(command == '1')
    {
        moveForward(1);        
        IRFunction();
    }
    else if(command == '2')
    {
        moveForward(2);
        IRFunction();
    }
    else if(command == '3')
    {
        moveForward(3);
        IRFunction();
    }
    else if(command == '4')
    {
        moveForward(4);
        IRFunction();
    }
    else if(command == '5')
    {
        moveForward(5);
        IRFunction();
    }
    else if(command == '6')
    {
        moveForward(6);
        IRFunction();
    }
    else if(command == '7')
    {
        moveForward(7);
        IRFunction();
    }
    else if(command == '8')
    {
        moveForward(8);
        IRFunction();
    }
    else if(command == '9')
    {
        moveForward(9);
        IRFunction();
    }
    else if(command == '0')
    {
        moveStop();
        IRFunction();
    }
    else if(command == 'R')
    {
        turnRight(90,150);
        IRFunction();
    }
    else if(command == 'L')
    {
        turnLeft(90,150);
        IRFunction();
    }
    else if (command == 'V')
    {
        IRFunction();
    }
    else if(command == 'C')
    {
        adjustDistance();
        calibrate();
        adjustDistance();
        calibrate();
        IRFunction();
    }
    else if(command == 'D')
    {
        adjustDistance();
        // adjustDistance();
        // calibrate();
        // IRFunction();
    }
}

int moveForward(int distance){
    int multiplier;
    switch(distance){
        case 1: multiplier = 570; break;
        case 2: multiplier = 580; break;
        case 3: multiplier = 590; break;
        case 4: multiplier = 590; break;
        case 5: multiplier = 590; break;
        case 6: multiplier = 590; break;
        case 7: multiplier = 590; break;
        case 8: multiplier = 590; break;
        case 9: multiplier = 590; break;
        default: multiplier = 590; break;
    }
    int target_Distance = multiplier * distance;

    int count=0;
    int pwm1=300, pwm2=300; 
    int output=0;
    int LeftPosition,RightPosition;

    we.getCountsAndResetM1();
    we.getCountsAndResetM2();

    while(1)
    {
        LeftPosition = we.getCountsM1();
        RightPosition = we.getCountsM2();

        //Acceleration
        if(LeftPosition <=200)
        {
            pwm1 = 200;
            pwm2 = 200;
        } 
        else if(LeftPosition <=300)
        {
            pwm1 = LeftPosition;
            pwm2 = RightPosition;

            // if(pwm1>295){
            if(pwm1>300){
                pwm1 = 300;
            }
            // if(pwm2>315){
            if(pwm2>300){
                pwm2 = 300;
            }
        } 
        else 
        {
            pwm1 = 300;//280;
            pwm2 = 300;//320;
        }   
        // 280 and 320 are some magic numbers.

        if(LeftPosition >= target_Distance)
        {
            moveStop();
            break;
        }
        debug("LeftPosition: ");   
        debug(LeftPosition);
        debug(", target_Distance: ");   
        debug(target_Distance);
        
        if(distance == 1)
        {
            if(LeftPosition >= (target_Distance-150))
            {
                pwm1 = target_Distance-LeftPosition;
                pwm2 = target_Distance-LeftPosition;
            }
        }

        output = pidControlForward(we.getCountsM1(),we.getCountsM2());
        md.setSpeeds(pwm1+output, pwm2-output);

        debug(" pwm1: ");   
        debug(pwm1-output);
        debug(", pwm2: ");   
        debug(pwm2+output);
        debug(", output: ");   
        debug(output);
        debug(", motor1_encoder: ");   
        debug(we.getCountsM1());
        debug(", motor2_encoder: ");   
        debug(we.getCountsM2());
        debugNL();
    }
  
}

int pidControlForward(int LeftPosition, int RightPosition){
    float error = LeftPosition - RightPosition;
    return error*1.00;
}

int turnLeft(int degree, int power){
    return movement((int)(turn1Count*degree*errorLeft), 250, true, false);
}

int turnRight(int degree, int power){
    return movement((int)(turn1Count*degree*errorRight), 250, false, true);
}

//Control the movement of the motor
int movement(int counts, double rpm, bool left, bool right){
  
    int motorRunning = 2;  //2 motors are running
    int directionM1 = 1, directionM2 = 1;  //Using to set moveing direction
    unsigned long nowTime = 0, lastTime = 0;
    int m1Counts = 0, m2Counts = 0;
    int m1LastCounts = 0, m2LastCounts = 0;

    double inputM1 = 0.0, outputM1 = 0.0, setpointM1 = leftMotorAdjust*rpm;
    double inputM2 = 0.0, outputM2 = 0.0, setpointM2 = rpm;
  
    PID m1PID(&inputM1, &outputM1, &setpointM1, 0.999, 0.900, 0, DIRECT);  //Right 0.0325
    PID m2PID(&inputM2, &outputM2, &setpointM2, 0.999, 0.900, 0, DIRECT);  //Left

    m1PID.SetMode(AUTOMATIC);
    m2PID.SetMode(AUTOMATIC);
    m1PID.SetSampleTime(pidLoopTime);
    m2PID.SetSampleTime(pidLoopTime);

    m1PID.SetOutputLimits(0, 300);
    m2PID.SetOutputLimits(0, 300);

    if(left) directionM1 = -1;
    if(right) directionM2 = -1;

    resetEncoderCount();
    lastTime = millis();
    moveStop();
  
    while(motorRunning > 0)
    {
        if(abs(we.getCountsM1()) >= counts)
        {
            motorRunning--;
        }
        if(abs(we.getCountsM2()) >= counts)
        {
            motorRunning--;
        }
        nowTime = millis();
        if(nowTime-lastTime >= pidLoopTime)
        { //Wait till pidLoopTime
            m1Counts = abs(we.getCountsM1());
            m2Counts = abs(we.getCountsM2());
            lastTime = millis();
            inputM1 = ((double)abs(m1Counts - m1LastCounts) * 60 * 1000 / pidLoopTime)/perRevolutionCount;
            inputM2 = ((double)abs(m2Counts - m2LastCounts) * 60 * 1000 / pidLoopTime)/perRevolutionCount;

            m1PID.Compute();
            m2PID.Compute();
            md.setM1Speed((int)outputM1 * directionM1);
            md.setM2Speed((int)outputM2 * directionM2);

            debug("speed1: "); debug(setpointM1);
            debug(", speed2: "); debug(setpointM2);
            debug(", speed1: "); debug(outputM1);
            debug(", speed2: "); debug(outputM2);
            debugNL();

            m1LastCounts = m1Counts;
            m2LastCounts = m2Counts;
        } 
    }
    moveStop();
}

void moveStop(){
    digitalWrite(2,LOW);
    digitalWrite(4,LOW);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    analogWrite(9,255);
    analogWrite(10,255);
    delay(200);
    analogWrite(9,1);
    analogWrite(10,1);
    delay(100);
}

void resetEncoderCount()
{
    we.getCountsAndResetM1();
    we.getCountsAndResetM2();
}

float getIR(int irPin , int sensorNo)
{
    float mydistance = 0.0;
    int numSample = 20;
    float arrdistance[numSample];
    float voltage_mv = 0.0;

    float result=0;
    int i=0;

    for(i=0; i<numSample; i++)
    {
        arrdistance[i] = 108553.481 * (float)pow((float)(analogRead(irPin) / 1023.0 * 5000), -1.2);
    }

    mydistance = mode(arrdistance,numSample);

    mydistance =(mydistance>50.0)?50.0:mydistance;
    return mydistance;
}

void IRFunction()
{
    float frontSensorOffset = -2.671777;
    //front right sensor
    float sensor1 = getIR(2,1);
    
    //front left sensor
    float sensor2 = getIR(3,2);

    //left sensor
    float sensor3 = getIR(4,3);

    //right sensor
    float sensor4 = getIR(5,4);

    //front sensor
    //do some trick to trick algorithm
    //because the sensor is moving backward now
    
    // float sensor5 = getFrontSensor();
    float sensor5 = getFrontSensor() + frontSensorOffset;
    if(sensor5<7.00){
        sensor5 = 7.00;
    }

    Serial.print("1,");
    Serial.print(sensor5);
    Serial.print(",");
    Serial.print(sensor2);
    Serial.print(",");
    Serial.print(sensor1);
    Serial.print(",");
    Serial.print(sensor3);
    Serial.print(",");
    Serial.println(sensor4); 
} 


//this variable is use to determine which two
//sensors to be used for calibration
//must call the function makeCalibrationDecision()
//before using it
//-1: cannot calibrate
//0: use front left and front right
//1: use front left and front
//2: use front and front right
int decision = -1;

void makeCalibrationDecision()
{
    float adjustDis = 16.00;
    float sensor1 = getFrontLeftSensor();
    float sensor2 = getFrontRightSensor();
    // float sensor3 = getFrontSensor();
    if(sensor1 <= adjustDis && sensor2 <= adjustDis)
    {
        decision = 0;
    }
    else
    {
        decision = -1;
    }
}

void calibrate()
{
    float limit = 0.05;
    float sensor1 = 0.0;
    float sensor2 = 0.0;
    float error = 0.0;
    float adjustDis = 16.00;
    float frontSensorOffset = 0.49;
    float frontLeftSensorOffset = -0.05;//0.22;

    makeCalibrationDecision();
    debug("Decision: ");
    debug(decision);
    debugNL();
    if(decision==-1)
    {
        return;
    }

    sensor1 = getFrontLeftSensor()+frontLeftSensorOffset;
    sensor2 = getFrontRightSensor();

    debug("FrontLeftSensor: ");
    debug(sensor1);
    debug(" FrontRightSensor: ");
    debug(sensor2);
    debugNL();
    for(int i = 0; i < 200; i++)
    {
        error = sensor1 - sensor2;
        debug("Error: ");
        debug(error);
        debugNL();

        if(abs(error)<limit)
        {
            debugNL("break here");
            moveStop();
            break;
        }
        if(error < 0)
        {
            debugNL("Move left");
            moveLeft();
        }
        else if(error < 20)
        {
            debugNL("Move right");
            moveRight();
        }
        else
        {
            debugNL("cannot calibrate, big error");
            moveStop();
            break;
        }

        sensor1 = getFrontLeftSensor()+frontLeftSensorOffset;
        sensor2 = getFrontRightSensor();

    }
}


int adjustDistance()
{
    int motorPower = 80;
    float goodDistance = 9.78;
    float limit = 0.10;
    float sensor3 = getFrontSensor();
    
    debug(", sensor3: ");
    debug(sensor3);
    debugNL();
    while(sensor3>20.0){   
        moveForward(1);
        sensor3 = getFrontSensor();
        debug(", sensor3: ");
        debug(sensor3);
        debugNL();
    }
    for(int j=0; j < 100; j++)
    {
        debug("sensor3: ");
        debug(sensor3);
        debug(", ");
        if(abs(sensor3 - goodDistance)<limit)
        {
            break;
        }
        else if(sensor3<goodDistance)
        {
            md.setSpeeds(-1*motorPower,-1*motorPower);
            debugNL("Move backward");
        }
        else
        {
            md.setSpeeds(motorPower,motorPower);
            debugNL("Move forward");
        }
        sensor3 = getFrontSensor();
    }
    moveStop();
    resetEncoderCount();

    return 1;
}

void moveLeft()
{
    int motorPower = 80;
    int revolutionNeeded = 0; //arduino:1400 assessment:1185
    int totalRevolution = 0;
    int m1Power = (-1 * motorPower);
    int m2Power = motorPower - 8;
    revolutionNeeded = 10;
    resetEncoderCount();

    while((abs(we.getCountsM1()) < revolutionNeeded) && (Serial.available() <= 0))
    {
        md.setSpeeds(m1Power, m2Power);
    }
    md.setBrakes(m1Power,m2Power);
    resetEncoderCount();
}

void moveRight()
{
    int motorPower = 80;
    int revolutionNeeded = 0; //Arduino:1310 assessment:1170
    int totalRevolution = 0; 
    int m1Power = motorPower;
    int m2Power = ( -1 * (motorPower + 1));
    revolutionNeeded = 10;
    resetEncoderCount();

    while((abs(we.getCountsM1()) < revolutionNeeded) && (Serial.available() <= 0))
    {
        md.setSpeeds(m1Power, m2Power);
    }
    md.setBrakes(m1Power,m2Power);
    resetEncoderCount(); 
}

//utility function
float getFrontSensor()
{
    return getIR(1,5);
}

float getFrontLeftSensor()
{
    return getIR(3,2);
}

float getFrontRightSensor()
{
    return getIR(2,1);
}

float getLeftSensor()
{
    return getIR(4,3);
}

float getRightSensor()
{
    return getIR(5,4);
}

void debug(String message)
{
    if(DEBUG)
        Serial.print(message);
}
void debug(int message)
{
    if(DEBUG)
        Serial.print(message);
}
void debug(float message)
{
    if(DEBUG)
        Serial.print(message);
}
void debug(double message)
{
    if(DEBUG)
        Serial.print(message);
}
void debug(char message)
{
    if(DEBUG)
        Serial.print(message);
}
void debugNL(){
    if(DEBUG)
        Serial.println();
}
void debugNL(String message){
    if(DEBUG)
        Serial.println(message);
}
void debugNL(float message){
    if(DEBUG)
        Serial.println(message);
}


