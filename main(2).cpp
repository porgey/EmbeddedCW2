#include "mbed.h"
#include "SHA256.h"
#include "inttypes.h"
#include "Mail.h"
#include "PwmOut.h"

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6           //0x08
#define L3Lpin D10          //0x10
#define L3Hpin D2           //0x20

#define PWMpin D9

//Test pins
#define T1pin D4

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Message Identifiers
#define ROTATION_SET    0;
#define VELOCITY_SET    1;
#define NEW_KEY_SET     2;
#define NEW_TUNE_SET    3;
#define CURR_VELOCITY   4;

//Global variables
int8_t intState = 0;
int8_t oldIntState = 0;
volatile int32_t rotorStateChange = 0;
bool newKeyIn = false;
uint64_t newKey;
float intVelError = 0.0;
float prevVelError = 0.0;

//Temporary PID constants
float velPropConst = 0.0;
float velIntConst = 0.0;
float velDiffConst = 0.0;
float rotPropConst = 10.0;
float rotDiffConst = 20.0;
float intErrConst = 0.01;
float intErrRan = 60.0;

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Origin state of motor
int8_t orState;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Test Pin output
DigitalOut T1T(T1pin);

//Motor PWM initialisation
PwmOut motor(PWMpin);
float motorPWM = 0.0;

//Initialise velocity variables
float setVelocity = 0.0;
float currVelocity = 0.0;

//Initialise rotation variables
float setRotation = 0.0;
float setRotTemp = 0.0;
float currRotation = 0.0;
float prevRotError = 0.0;

//Initialise hash rate
uint32_t hashRate = 0;

//Initialise Serial
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Initialise Threads
Thread outputThread(osPriorityNormal,1536);
Thread motorCtrlThread(osPriorityHigh,1024);
Thread decodeThread(osPriorityNormal,1536);

//Initialise Mutexes
Mutex newKey_mutex;

//test function
void test_trig(){
    if (T1T = 0) T1T = 1;
    else if(T1T = 1) T1T = 0;
}

/* Mail */
typedef struct {
    int32_t sendMessage;
    int8_t identifier;
} mail_t;

Mail<mail_t, 16> mail_box;

void putMessage(int8_t identifier, int32_t message){
    mail_t *mail = mail_box.alloc();
    mail->sendMessage = message;
    mail->identifier = identifier;
    mail_box.put(mail);
}

//Output thread
void output_thread(){
    while(true){
        osEvent evt = mail_box.get();
        mail_t *mail = (mail_t*)evt.value.p;
        if(mail->identifier == 0){      //Rotation set
            pc.printf("Rotation SET: %f\trotations\r\n", *(float*)&(mail->sendMessage));
        }
        else if(mail->identifier == 1){ //Current rotation
            pc.printf("Current Rotation: %f \t\r\n", *(float*)&(mail->sendMessage));
        }
        else if(mail->identifier == 2){ //Velocity set
            pc.printf("Velocity SET: %f\trps\r\n", *(float*)&(mail->sendMessage));
        }
        else if(mail->identifier == 3){ //Current velocity
            pc.printf("Current Velocity: %f\t\trps\r\n", *(float*)&(mail->sendMessage));
        }
        else if(mail->identifier == 4){ //Upper half of key
            pc.printf("Key SET: 0x%X", mail->sendMessage);
        }
        else if(mail->identifier == 5){ //Lower half of key
            pc.printf("%X\r\n", mail->sendMessage);
        }
        else if(mail->identifier == 6){ //Upper half of Nonce
            pc.printf("Successful Nonce: 0x%X", mail->sendMessage);
        }
        else if(mail->identifier == 7){ //Lower half of Nonce
            pc.printf("%X\r\n", mail->sendMessage);
        }
        else if(mail->identifier == 8){ //Hash rate
            pc.printf("Current Hash Rate: %d\r\n", mail->sendMessage);
        }
        else if(mail->identifier == 9){ //Hash rate
            pc.printf("Torque: %f\r\n", *(float*)&(mail->sendMessage));
        }
        
        mail_box.free(mail);
    }
}

Queue<void, 8> inCharQ;
char command[17];
int commandIndex = 0;

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}
//thread to decode incoming serial commands
void decode_thread(){
    pc.attach(&serialISR);
    while(true) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;

        if(commandIndex > 18){
            commandIndex = 0;
        }
        else{
            command[commandIndex] = newChar;
        }
        if(newChar == '\r') {
            
            command[commandIndex]='\0';
            commandIndex = 0;

            if(command[0] == 'R'){
                sscanf(command,"R%f",&setRotTemp);
                putMessage(0, *(int32_t*)&(setRotTemp));
                setRotation = currRotation + setRotTemp;
            }
            else if(command[0] == 'V'){
                sscanf(command,"V%f",&setVelocity);
                putMessage(2,*(int32_t*)&(setVelocity));
            }
            else if(command[0] == 'K'){
                sscanf(command,"K%x",&newKey);
                putMessage(4, (uint32_t)((newKey>>32)&0xFFFFFFFF));
                putMessage(5, (uint32_t)(newKey&0xFFFFFFFF));
                newKeyIn = true;
            }
            else if(command[0] == 'T'){
            }
            else{
            }
        }
        else{
            commandIndex++;
        }
    }
}

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L  =1;
    if (driveOut & 0x20) L3H = 0;
    
}

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

//attached to photointerrputers
void motorISR(){
    int8_t intState = readRotorState();
    motorOut((intState-orState+lead+6) % 6); //+6 to make sure the remainder is positive
    
    if(intState - oldIntState == -5){//for change from 5 to 0
        rotorStateChange++;
    }
    else if(intState - oldIntState == 5){//for change from 0 to 5
        rotorStateChange--;
    }
    else{
        rotorStateChange += (intState - oldIntState); //increment / decrement variable
    }
    oldIntState = intState;
}

//return torque for motorvelocity
float motorVelocity(){
    
    //e_s
    float velError = setVelocity - abs(currVelocity);

    float diffVelError = velError - prevVelError;
    
    
    intVelError +=  velError*0.05;
    if(intVelError > 35.0) intVelError = 35.0;
    if(intVelError < -35.0) intVelError = -35.0;

    float torque = 25*velError + 0*diffVelError + 5*intVelError;
    
    if(torque > 1.0){
        torque = 1.0;
    }
    else if(torque < -1.0){
        torque = -1.0;
    }
//    putMessage(3, *(int32_t*)&currVelocity);
    if(torque >= 0.0){
        lead = 2;
    }
    else{
        lead = 0;
        torque = -torque;
    }
    return torque;
}

//return torque for motorposition
float motorPosition(){
    
    //e_r
    float rotError = setRotation - currRotation;
    
    float diffRotError = rotError - prevRotError;
    prevRotError = rotError;
    
    float torque = 10*rotError + 20*diffRotError;
    
    if(torque > 1.0){
        torque = 1.0;
    }
    else if(torque < -1.0){
        torque = -1.0;
    }
//    putMessage(3, *(int32_t*)&currVelocity);
    //set lead to 0 if close to correct position
    if(abs(rotError) <= 0.17){
        lead = 0;
        torque = 0.0;
    }
    //reverse lead to chsnge direction if necessary
    else if(torque >= 0.0){
        lead = 2;
    }
    else{
        lead = -2;
        torque = -torque;
    }
    return torque;
}
//tick to execute motorCtrl_thread
void motorCtrlTick(){
    motorCtrlThread.signal_set(0x1);
}

void motorCtrl_thread(){
    
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    
    uint8_t velocityIterations = 1;
    
    float oldRotorState = 0.0;
    float prevVelocityError = 0.0;
    int32_t rotorStateChangeNow = 0;

    while(true){
        
        oldRotorState = rotorStateChange;
        
        motorCtrlThread.signal_wait(0x1);
        
        //criticsl section to extract totorStateChange
        core_util_critical_section_enter();
        rotorStateChangeNow = rotorStateChange;
        core_util_critical_section_exit();
        
        //calculating current velocity and current rotations
        currVelocity = 10.0*(((float)rotorStateChangeNow)/6.0-((float)oldRotorState/6.0));
        currRotation = (float)rotorStateChangeNow/6.0;
        
        velocityIterations += 1;
        if(velocityIterations == 10){
            velocityIterations = 0;
            //print current velocity and rotations
            putMessage(3, *(int32_t*)&currVelocity);
            putMessage(1, *(int32_t*)&currRotation);
        }

        //start motor if stationary and required to move
        if(currVelocity == 0 && (setVelocity || setRotation)){
            int8_t intState = readRotorState();
            motorOut((intState-orState+lead+6) % 6); //+6 to make sure the remainder is positive
        }
        //uncomment pair for torque for velocity or position
        //clearly not the correct implementation
        float T1 = motorVelocity();
        motorPWM = T1;
//        float T2 = motorPosition();
//        motorPWM = T2;
        
    }
}

//Prints hash rate every second
void hashCount(){
    putMessage(8,hashRate);
    hashRate = 0;
}

//Main
int main() {
    //set PWM for motorHome
    motor.period(0.002f);
    motorPWM = 1;
    motor.write(motorPWM);
    
    //Run the motor synchronisation
    orState = motorHome();
    oldIntState = orState;
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    //attach motorISR to photointerrupters
    I1.rise(&motorISR);
    I1.fall(&motorISR);
    I2.rise(&motorISR);
    I2.fall(&motorISR);
    I3.rise(&motorISR);
    I3.fall(&motorISR);
    
//    I1.rise(&test_trig);
    
    //attach thread functions to threads
    outputThread.start(output_thread);
    motorCtrlThread.start(motorCtrl_thread);
    decodeThread.start(decode_thread);
    
    //Initialise mining
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    Ticker hashTicker;
    hashTicker.attach(&hashCount,1.0);
    
    while (true) {
        if(newKeyIn){
            newKey_mutex.lock();
            *key = newKey;
            newKey_mutex.unlock();
            newKeyIn = false;
        }
        
        SHA256::computeHash(&hash[0], &sequence[0], sizeof(sequence));
        if((hash[0]==0)&&(hash[1] == 0)){
            putMessage(6,(uint32_t)((*nonce>>32)&0xFFFFFFFF));
            putMessage(7,(uint32_t)((*nonce)&0xFFFFFFFF));
        }
        *nonce+=1;
        hashRate++;
        //write PWM to motor     
        motor.write(motorPWM);
    }
}

