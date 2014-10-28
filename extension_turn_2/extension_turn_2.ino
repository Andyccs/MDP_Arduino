#include <PololuWheelEncoders.h>
#include <DualVNH5019MotorShield.h>
#include <Average.h>
#include <PID_v1.h>

/*====================|| Motors ||====================*/
PololuWheelEncoders we;
DualVNH5019MotorShield md;

// added by xinzi on 23 Oct start
int front_count = 568; // 562
double motor_ajust = 0.975; //0.965;;//0.965;//0.975;
// added by xinzi on 23 Oct end

//int perRevolutionCount = 1092; //1484;//2245; //1024;
int perRevolutionCount = 1097;//1092; 
/*
int perRevolutionCountLeft = 1120; 
int perRevolutionCountRight = 1300; 
*/
int wheelDiameter = 6;
int oneGrid10cm = (10 / (PI * wheelDiameter)) * perRevolutionCount;  //550 for 10cm
double robotDiameter = 17.0;  //17.0;
double turn90Count = (((((PI/2.0) * (robotDiameter/2.0)) / (PI*wheelDiameter)) * perRevolutionCount));
double turn1Count = turn90Count/90;
double turn3Deg = (((((PI/2.0) * (0.3/2.0)) / (PI*wheelDiameter)) * perRevolutionCount));
//double errorRight = 0.999;
//double errorRight = 0.977;
//double errorLeft = 0.999;
double errorRight = 1.025;//1.0185; //0.980;
double errorLeft = 1.005;
double normalRPM = 40.0;
int pidLoopTime = 100;  //Determines how often the PID algorithm evaluates. The default is 200mS.
int motorPower = 250;  //Set motor power/speed
int motor2Offset = 0;
int motorDegPower = 25;  //Power use to adjust
int gribToMove = 1;  //Deafult is 1 for exploration and will change accordingly during fastest path


int const URPWM = 6; // PWM Output 0－25000US，Every 50US represent 1cm
int const URTRIG = 12; // PWM trigger pin
volatile unsigned int Distance=0;
/*====================================================*/


  




boolean start = true;
boolean done = false;
boolean range = false;

int time = 180000; //3mins

int currentTime = 0;
int startTime = 0;

int motor1_a = 4;
int motor1_b = 2;
int motor2_a = 8;
int motor2_b = 7;

boolean fp = false;

/*=================|| Serial Read ||==================*/
char inData[20];  // Allocate some space for the string
char inChar = -1;  // Where to store the character read
byte index = 0;  // Index into array; where to store the character

//Serial Read String Function
char Comp(){
    // while(Serial.available() > 0)
    // {  // Don't read unless you know there is data
    //     if(index < 19)
    //     {  // One less than the size of the array   
    //         inChar = Serial.read(); // Read a character
    //         inData[index] = inChar; // Store it
    //         index++; // Increment where to write next
    //         inData[index] = '\0'; // Null terminate the string
    //     }
    // }
    // if(strcmp(inData, This) == 0)
    // {
    //     for (int i = 0; i < 19; i++)
    //     {
    //         inData[i] = 0;
    //     }
    //     index = 0;
    //     return(1);
    // }
    // else
    // {
    //     return(0);
    // }

    if(Serial.available()>0){
        return Serial.read();
    }else{
        return '\0';
    }
}

String dir, dis;
int dist;

void setup()
{
    md.init();  
    we.init(11,13,3,5);
    //PWM_Mode_Setup();
  
    Serial.begin(9600);
}

void loop(){
    
    char command = Comp();
    if(command == '1')
    {
        moveForward(1);
        //moveOneGrid();
        
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
    else if(command == '0')
    {
        moveStop();
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
}

// int moveForward(int grid, int power){
//     debug("rpm normal ");
//     debug(normalRPM);
//     debug(" power");
//     debug(power);
//     debugNL();
//     //return movement(front_count*grid, normalRPM+normalRPM*(double)power, false, false);
//     return movement(front_count*grid, 250, false, false);
// }
int prev_error; 
int integral;

int moveForward(int distance){
    int multiplier;
    switch(distance){
    //    case 1: multiplier = 1162; break;
        case 1: multiplier = 550; break;
        case 2: multiplier = 550; break;
        case 3: multiplier = 550; break;
        case 4: multiplier = 550; break;
        case 10: multiplier = 550; break;
        case 11: multiplier = 550; break;
        case 12: multiplier = 550; break;
        default: multiplier = 550; break;
    }
    int target_Distance = multiplier * distance;

    int count=0;
    int pwm1=300, pwm2=300; 
    int output=0;
    int LeftPosition,RightPosition;

    we.getCountsAndResetM1();
    we.getCountsAndResetM2();

    prev_error = 0;
    integral = 0;

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

            if(pwm1>280){
                pwm1 = 280;
            }
            if(pwm2>320){
                pwm2 = 320;
            }
        } 
        else 
        {
            pwm1 = 280;
            pwm2 = 320;
        }   

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
        // md.setSpeeds(pwm1-output+left_offset, pwm2+output);
        // md.setSpeeds(pwm1-output, pwm2+output);
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



// int pidControlForward(int LeftPosition, int RightPosition){
//     int error;
//     float derivative,output;

//     //0.75
//     float Kp = 0.75;  //0-0.1

//     //1.65
//     float Kd = 1.65;  //1-2

//     //0.65
//     float Ki = 0.75;  //0.5-1

//     error = LeftPosition - RightPosition;
//     integral += error;
//     derivative = (error - prev_error);
//     output = Kp*error + Ki * integral + Kd * derivative;
//     prev_error = error;

//     debug(", error: ");
//     debug(error);
//     debug(", integral: ");
//     debug(integral);
//     debug(", derivative: ");
//     debug(derivative);


//     return output;
// }

int pidControlForward(int LeftPosition, int RightPosition){
    // float error;
    // int prev_error,pwm1=255,pwm2=255;
    // float integral,derivative,output;
    // //0.75
    // float Kp = 0.75;  //0-0.1

    // //1.65
    // float Kd = 1.65;  //1-2

    // //0.65
    // float Ki = 0;  //0.5-1

    // error = LeftPosition - RightPosition;
    // integral += error;
    // derivative = (error - prev_error);
    // output = Kp*error + Ki * integral + Kd * derivative;
    // prev_error = error;

    // //Serial.println(error);

    // debug(", error: ");
    // debug(error);
    // debug(", integral: ");
    // debug(integral);
    // debug(", derivative: ");
    // debug(derivative);

    // pwm1=output;
    // return pwm1;

    float error = LeftPosition - RightPosition;
//    if(LeftPosition<100.0){
//        return (int) error * 1.9;
//    }
//    else if(LeftPosition<250.0){
//        return (int) error * 1.6;
//    }
//    else{
//        return (int) error * 1.3;
//    }
    return error*1.00;
}

// void moveOneGrid()
// {
//     int motorPower = 200;
//     int m1Power = motorPower-20;//motorPower-2.90;
//     int m2Power = motorPower;

//     int gap = 7;
//     int error = 0;
//     int aggkp = 2;
//     int goodDistance = 470;
//     int totalDistance = 0;
//     int errorOffsetM1 = 0;
//     int errorOffsetM2 = 0;
//     bool slowState = false;

//     const int smallErrorPower = 5;
//     const int midErrorPower = 7;
//     const int bigErrorPower = 9;

//     we.getCountsAndResetM1();
//     we.getCountsAndResetM2();

//     while(true){

//         if(goodDistance-we.getCountsM1()<5)
//         {  
//             debug("done: ");
//             debug(goodDistance-we.getCountsM1());
//             debugNL();
//             break;
//         }
//         else
//         {
//             md.setM1Speed(m1Power);
//             md.setM2Speed(m2Power);
//             debug("fast motion, ");
//         }

//         debug("m1Power: ");
//         debug(m1Power);
//         debug(", m2Power: ");
//         debug(m2Power);
//         debug(", countM1: ");
//         debug(we.getCountsM1());
//         debug(", countM2: ");
//         debug(we.getCountsM2());
//         debug(", errorOffsetM1: ");
//         debug(errorOffsetM1);
//         debug(", errorOffsetM2: ");
//         debug(errorOffsetM2);

//         //determine the error after deducting from the 2 motor
//         // error = (we.getCountsM1()-errorOffsetM1) - (we.getCountsM2()-errorOffsetM2);
//         error = we.getCountsM1() - we.getCountsM2();
//         debug(", error: ");
//         debug(error);
        


//         if(abs(error) > 3)
//         {
//             if(error<0)
//             {
//                 debug(", addM2 ");
//                 if(we.getCountsM1()-we.getCountsM2()<=10)
//                 {
//                     debug(" small error");
//                     m2Power+=smallErrorPower;
//                 }
//                 else if(we.getCountsM1()-we.getCountsM2()<=20)
//                 {
//                     debug(" medium error");
//                     m2Power+=midErrorPower;
//                 }
//                 else
//                 {
//                     debug(" big error");
//                     m2Power+=bigErrorPower;
//                 }
//             }
//             else if(error>0)
//             {
//                 debug(", addM1 ");
//                 if(we.getCountsM2()-we.getCountsM1()<=10)
//                 {
//                     debug(" small error");
//                     m1Power+=smallErrorPower;
//                 }
//                 else if(we.getCountsM2()-we.getCountsM1()<=20)
//                 {
//                     debug(" medium error");
//                     m1Power+=midErrorPower;
//                 }
//                 else
//                 {
//                     debug(" big error");
//                     m1Power+=bigErrorPower;
//                 }
//             }
//         }
//         debugNL();

//     }

//     moveStop();
// }


int moveBackward(int grid, int power){
    return movement(front_count*grid, normalRPM+normalRPM*(double)power, true, true);
}

int turnLeft(int degree, int power){
    // oneGrid10cm = (10 / (PI * wheelDiameter)) * perRevolutionCountLeft;
    //return movement((int)(turn1Count*degree*errorLeft), (normalRPM+normalRPM*(double)power), true, false);
    return movement((int)(turn1Count*degree*errorLeft), 250, true, false);
}

int turnRight(int degree, int power){
    // oneGrid10cm = (10 / (PI * wheelDiameter)) * perRevolutionCountRight;
    //return movement((int)(turn1Count*degree*errorRight), (normalRPM+normalRPM*(double)power), false, true);
    return movement((int)(turn1Count*degree*errorRight), 250, false, true);
}

// int turn3DegLeft(int degPower){
//     return movement((int)(turn3Deg*90*errorLeft), (normalRPM+normalRPM*(double)degPower), true, false);
// }

// int turn3DegRight(int degPower){
//     return movement((int)(turn3Deg*90*errorRight), (normalRPM+normalRPM*(double)degPower), false, true);
// }

//Control the movement of the motor
int movement(int counts, double rpm, bool left, bool right){
  
    int motorRunning = 2;  //2 motors are running
    int directionM1 = 1, directionM2 = 1;  //Using to set moveing direction
    unsigned long nowTime = 0, lastTime = 0;
    int m1Counts = 0, m2Counts = 0;
    int m1LastCounts = 0, m2LastCounts = 0;

    double inputM1 = 0.0, outputM1 = 0.0, setpointM1 = motor_ajust*rpm;
    double inputM2 = 0.0, outputM2 = 0.0, setpointM2 = rpm;

    // PID m1PID(&inputM1, &outputM1, &setpointM1, 0.150, 0.002, 0, DIRECT);  //Right 0.0325
    // PID m2PID(&inputM2, &outputM2, &setpointM2, 0.999, 0.900, 0, DIRECT);  //Left
  
    // // if(counts = oneGrid10cm)
    // if(left==false && right==false)
    // {
    //     // m1PID.SetTunings(0.150, 0.002, 0);
    //     m1PID.SetTunings(0.900, 0.900, 0);
    //     m2PID.SetTunings(0.900, 0.900, 0);
    // }
  
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
    float sensor1 = getIR(2,1);
    
    //front left sensor
    float sensor2 = getIR(3,2);

    //left sensor
    float sensor3 = getIR(4,3);

    //right sensor
    float sensor4 = getIR(5,4);

    //front sensor
    // float sensor5 = getIR(1,5);
    float sensor5 = getFrontSensor();

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
    float sensor3 = getFrontSensor();
    if(sensor1 <= adjustDis && sensor2 <= adjustDis)
    {
        decision = 0;
    }
    // else if(sensor1 <= adjustDis && sensor3 <= adjustDis)
    // {
    //     decision = 1;
    // }
    // else if(sensor2 <= adjustDis && sensor3 <= adjustDis)
    // {
    //     decision = 2;
    // }
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
    float frontLeftSensorOffset = 0.1;

    makeCalibrationDecision();
    debug("Decision: ");
    debug(decision);
    debugNL();
    if(decision==-1)
    {
        return;
    }
    else if(decision==0)
    {
        sensor1 = getFrontLeftSensor()+frontLeftSensorOffset;
        sensor2 = getFrontRightSensor();
    }
    else if(decision==1)
    {
        sensor1 = getFrontLeftSensor();
        sensor2 = getFrontSensor()+frontSensorOffset;
    }
    else if(decision==2)
    {
        sensor1 = getFrontSensor()+frontSensorOffset;
        sensor2 = getFrontRightSensor();
    }
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

        //get correct sensor information according to decision
        if(decision==0)
        {
            sensor1 = getFrontLeftSensor()+frontLeftSensorOffset;
            sensor2 = getFrontRightSensor();
        }
        else if(decision==1)
        {
            sensor1 = getFrontLeftSensor();
            sensor2 = getFrontSensor()+frontSensorOffset;
        }
        else if(decision==2)
        {
            sensor1 = getFrontSensor()+frontSensorOffset;
            sensor2 = getFrontRightSensor();
        }
    }
}


int adjustDistance()
{
    int motorPower = 80;
    float disAdjust = 10.0;
    float goodDistance = 7.4;
    float limit = 0.05;

    float sensor1 = getFrontLeftSensor();
    float sensor2 = getFrontRightSensor();
    float sensor3 = getFrontSensor();
    
    debug("sensor1: ");
    debug(sensor1);
    debug(", sensor2: ");
    debug(sensor2);
    debug(", sensor3: ");
    debug(sensor3);
    debugNL();
    while(sensor1>20.0 && sensor2>20.0 && sensor3>20.0){
        moveForward(1);
        sensor1 = getFrontLeftSensor();
        sensor2 = getFrontRightSensor();
        sensor3 = getFrontSensor();
        debug("sensor1: ");
        debug(sensor1);
        debug(", sensor2: ");
        debug(sensor2);
        debug(", sensor3: ");
        debug(sensor3);
        debugNL();
    }
    for(int i =0; i < 200; i++)
    {
        //if the distance is nice
        if(abs(sensor1 - goodDistance)<limit || 
            abs(sensor2 - goodDistance)<limit ||
            abs(sensor3 + 0.27 - goodDistance)<limit)
        {
            break;
        }
        //check if far
        else if(sensor1>goodDistance && sensor2>goodDistance && sensor3>goodDistance)
        {
            //move forward
            md.setSpeeds(motorPower-8,motorPower);
            debugNL("Move forward");
        }
        //check if near
        else
        {
            //move backward
            md.setSpeeds(-1*(motorPower-8),-1*motorPower);
            debugNL("Move backward");
        }
        sensor1 = getFrontLeftSensor();
        sensor2 = getFrontRightSensor();
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
//For debugging purpose
bool DEBUG = true;
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


