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
double errorRight = 1.0; // 1060

//this value can be used to increase or decrease 
//the rpm when turn left
//change to higher value for more rotation
//change to lower value for less rotation
double errorLeft = 1.0; // 0999

//Determines how often the PID algorithm evaluates. The default is 200mS.
//do not change the value
const int pidLoopTime = 100;  

//do not change it, offset has been set.
const int GOOD_DISTANCE = 500;

//this number will be used to see when to 
//slow down allign distance from far to near
//can change, should be abit larger than GOOD_DISTANCE
const int FRONT_TOLERANCE = 526;

//this number will be used to see when to 
//slow down allign distance from near to far
//can change, should be abit smaller than GOOD_DISTANCE
const int BACK_TOLERANCE = 484;

//this constant will be used when the robot move forward
//changing this can make the robot move straight
//plus to turn right 
//minus to turn left
const int MOVE_FORWARD_LEFT_RIGHT_OFFSET = 10;

//For debugging purpose
//set it to true to debug
//set it to false for production
const bool DEBUG = false;

//below are numbers to be set during initialization
//sensor raw read data offset
//sensor raw data will be set to 500
int initFrontLeftOffset = 0;
int initFrontMidOffset = 0;
int initFrontRightOffset = 0;
//moveforward multiplier
int initMultiplier[9] = {500,500,500,500,500, 500,500,500,500};
// 1..9
int initForwardLeftOffset[9] = {0,0,0,0,0, 0,0,0,0};
// a..i
// IR offest
float initIRFront = 0.0;
float initIRFrontLeft = 0.0;
float initIRFrontRight = 0.0;


//pin number for motor
//do not change
const int motor1_a = 4;
const int motor1_b = 2;
const int motor2_a = 8;
const int motor2_b = 7;

//Serial Read String Function
char serialRead(){
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

int readStatus = 0;
char readVariable = 0;
int readValue = 0;

void loop(){
    char command = serialRead();
    if (command == '\0') return;
    if (readStatus > 0) {
        //first state of the setting
        //Reading what to setting
        if (readStatus == 1)
        {
            readVariable = command;
            readStatus = 2;
            readValue = 0;
        }

        //second state of the setting
        //read in the value and make action
        else if (readStatus == 2)
        {
            //if we reach the end of setting command
            //do appropriate action
            if (command == 'S' || command == 'T')
            {
                if (readVariable == 'L')
                {
                    errorLeft = 1.0 * readValue / 1000.0;
                }
                else if (readVariable == 'R')
                {
                    errorRight = 1.0 * readValue / 1000.0;
                }
                else if (readVariable <= '9' && readVariable >= '0')
                {
                    initMultiplier[readVariable - '1'] = readValue;
                }
                else if (readVariable <= 'i' readVariable >= 'a')
                {
                    initForwardLeftOffset[readVariable - 'a'] = readValue
                }
                if (command == 'S') {
                    // done reading
                    Serial.print("Try to set constant ");
                    Serial.print(readVariable);
                    Serial.print(" to ");
                    Serial.print(readValue);
                    Serial.println();
                    readStatus = 0;
                } else {
                    readStatus = 1;
                }
            }
            //we continue reading the value to be set
            else
            {
                readValue *= 10;
                readValue += command - '0';
            }
        }

        return;
    }
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
        IRFunction();
    }
    else if(command == 'A')
    {
        calibrate();
        IRFunction();
    }
    else if(command == 'F')
    {
       feedBackFunction();
    }
    else if(command == 'I')
    {
        // Initialize
        initializeRobot();
    }
    else if(command == 'S')
    {
        //start reading setting information
        readStatus = 1;
    }
    
}

int moveForward(int distance){
    we.getCountsAndResetM1();
    we.getCountsAndResetM2();

    int multiplier = initMultiplier[distance-1];
    int target_Distance = multiplier * distance;

    int left_offset = initForwardLeftOffset[distance-1];

    int count=0;
    int pwm1=300, pwm2=300; 
    int output=0;
    int LeftPosition,RightPosition;

    while(1){
        LeftPosition = we.getCountsM1();    //hardcoded
        RightPosition = we.getCountsM2();  

        //Acceleration
        if(LeftPosition <=100){
            pwm1 = 100;
            pwm2 = 100;
        } 
        else if(LeftPosition <=300){
            pwm1 = LeftPosition;
            pwm2 = RightPosition;
        } 
        else {
           pwm1 = 300;
            pwm2 = 300;
        }   

        if(LeftPosition >= target_Distance-70){
            brake();
            break;
        }

        if(distance == 1){

            if(LeftPosition >= (target_Distance-70-200) && LeftPosition <= (target_Distance+100)){
                pwm1 = target_Distance-70-LeftPosition+100;
                pwm2 = target_Distance-70-LeftPosition+100;
            }
        }

        output = pidControlForward(we.getCountsM1(),we.getCountsM2());
        md.setSpeeds(pwm1+output+left_offset, pwm2-output);
    }

}

int pidControlForward(int LeftPosition, int RightPosition){
    int error;
    int prev_error;
    float integral,derivative,output;
    float Kp = 0.75;
    float Kd = 1.65;
    float Ki = 0;

    error = LeftPosition - RightPosition;
    integral += error;
    derivative = (error - prev_error);
    output = Kp*error + Ki * integral + Kd * derivative;
    prev_error = error;

    return output*3;
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
    //front right sensor
    float sensor1 = getIR(2,1) + initIRFrontRight;
    
    //front left sensor
    float sensor2 = getIR(3,2) + initIRFrontLeft;

    //left sensor
    float sensor3 = getIR(4,3);

    //right sensor
    float sensor4 = getIR(5,4);
    
    // float sensor5 = getFrontSensor();
    float sensor5 = getIR(1,5) + initIRFront;

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

void calibrate()
{
    int limit = 5;
    int sensor1 = 0;
    int sensor2 = 0;
    int error = 0;

    sensor1 = getFrontLeftSensor();
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
            break;
        }
        if(error < 0)
        {
            debugNL("Move left");
            moveRight();
        }
        else
        {
            debugNL("Move right");
            moveLeft();
        }
        sensor1 = getFrontLeftSensor();
        sensor2 = getFrontRightSensor();

    }
    brake();
}

void brake(){
    md.setBrakes(400, 400);
    delay(50);
    md.setBrakes(0, 0);
}

int adjustDistance()
{
    const int motorPower = 100;
    const int limit = 7;

    int sensor3 = getFrontSensor();

    debug("frontSensorFeedback: ");
    debug(sensor3);
    debugNL();

    while(sensor3<270){   
        moveForward(1);
        sensor3 = getFrontSensor();
        debug("frontSensorFeedback: ");
        debug(sensor3);
        debugNL();
    }
    for(int j=0; j < 100; j++)
    {
        debug("frontSensorFeedback: ");
        debug(sensor3);
        debug(", ");
        if(abs(sensor3 - GOOD_DISTANCE)<limit)
        {
            break;
        }
        else if(sensor3>BACK_TOLERANCE-20 && sensor3<FRONT_TOLERANCE+20){
            if(sensor3>GOOD_DISTANCE){
                debugNL("Move Backward Less");
                moveAbit(false,true);
            }else{
                debugNL("Move Forward Less");
                moveAbit(true,true);
            }
        }
        else if(sensor3>GOOD_DISTANCE)
        {
            debugNL("Move Backward More");
            moveAbit(false,false);
        }
        else
        {
            debugNL("Move Forward More");
            moveAbit(true,false);
        }

        sensor3 = getFrontSensor();
    }
    brake();
    resetEncoderCount();

    return 1;
}

void moveAbit(bool forward,bool less)
{
    int motorPower = 100;
    
    if(!forward){
        motorPower = -motorPower; 
    }
    
    int m1Power = motorPower+2;
    int m2Power = motorPower-2;
    resetEncoderCount();

    md.setSpeeds(m1Power, m2Power);
    if(less){
        while((abs(we.getCountsM1()) < 3))
        {
            md.setSpeeds(m1Power, m2Power);
        }
        brake();
    }else{
        md.setSpeeds(m1Power, m2Power);
    }
    resetEncoderCount();
}

void moveLeft()
{
    int motorPower = 100;
    int revolutionNeeded = 5;
    int m1Power = -motorPower;
    int m2Power = motorPower;
    resetEncoderCount();

    while((abs(we.getCountsM1()) < revolutionNeeded))
    {
        md.setSpeeds(m1Power, m2Power);
    }
    brake();
    resetEncoderCount();
}

void moveRight()
{
    int motorPower = 100;
    int revolutionNeeded = 5;
    int m1Power = motorPower;
    int m2Power = -motorPower;
    resetEncoderCount();

    while((abs(we.getCountsM1()) < revolutionNeeded))
    {
        md.setSpeeds(m1Power, m2Power);
    }
    brake();
    resetEncoderCount(); 
}

void insertionsort(int array[], int length){
  int i,j;
  int temp;
  for(i = 1; i < length; i++){
    for(j = i; j > 0; j--){
      if(array[j] < array[j-1]){
        temp = array[j];
        array[j] = array[j-1];
        array[j-1] = temp;
      }
      else
        break;
    }
  }
}

int getAverageFeedback(int pin){
    int sensors[30];
    int sum = 0;
    int average = 0;

    for(int i=0;i<30;i++){
        sensors[i] = analogRead(pin);
    }

    insertionsort(sensors,30);

    for(int i = 7; i < 22; i++){
        sum = sum + sensors[i];
    }
    average = sum/15;

    //sort
    return average;
}

int getTripleAverageFeedback(int pin){
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += getAverageFeedback(pin);
    }
    return sum/10;
}

//utility function
float getFrontSensor()
{
    return getAverageFeedback(1) + initFrontMidOffset;
}

float getFrontLeftSensor()
{
    return getAverageFeedback(3) + initFrontLeftOffset;
}

float getFrontRightSensor()
{
    return getAverageFeedback(2) + initFrontRightOffset;
}

float getLeftSensor()
{
    return getAverageFeedback(4);
}

float getRightSensor()
{
    return getAverageFeedback(5);
}

void feedBackFunction(){
    debug(getFrontSensor());
    debug(", ");
    debug(getFrontLeftSensor());
    debug(", ");
    debug(getFrontRightSensor());
    debug(", ");
    debug(getLeftSensor());
    debug(", ");
    debug(getRightSensor());
    debugNL();
}

float getAverageIR(int i, int j)
{
    float sum = 0.0;
    for (int i = 0; i < 10; i ++) {
        sum += getIR(i, j);
    }
    return sum / 10.0;
}

void initializeRobot()
{
    debug("Start initializing sensor");
    debugNL();
    initFrontMidOffset = 500 - getTripleAverageFeedback(1);
    initFrontLeftOffset =  500 - getTripleAverageFeedback(3);
    initFrontRightOffset = 500 - getTripleAverageFeedback(2);


    //front right sensor
    initIRFrontRight = 7.0 - getAverageIR(2,1);
    
    //front left sensor
    initIRFrontLeft = 7.0 - getAverageIR(3,2);

    // float sensor5 = getFrontSensor();
    initIRFront = 7.0 - getAverageIR(1,5);


    Serial.print("Initialize sensor done: ");
    Serial.print(initFrontMidOffset);
    Serial.print(", ");
    Serial.print(initFrontLeftOffset);
    Serial.print(", ");
    Serial.print(initFrontRightOffset);
    Serial.print(" IR return offset: ");
    Serial.print(initIRFront);
    Serial.print(", ");
    Serial.print(initIRFrontLeft);
    Serial.print(", ");
    Serial.print(initIRFrontRight);
    Serial.println();
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
