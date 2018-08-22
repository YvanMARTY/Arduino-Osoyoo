/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins
#define dir1PinL  2    //Motor direction
#define dir2PinL  4    //Motor direction
#define speedPinL 6    // Needs to be a PWM pin to be able to control motor speed

#define dir1PinR  7    //Motor direction
#define dir2PinR  8   //Motor direction
#define speedPinR 5    // Needs to be a PWM pin to be able to control motor speed

#define IR_PIN  13 //IR receiver Signal pin connect to Arduino pin 3
#define IR_em 3 //IR emission signal

/*From left to right, connect to D3,A1-A3 ,D10*/
/* 
#define LFSensor_0  3 
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 10 
*/

/* Capteur depassement ligne */
/*
 * Capteurs avants : A1
 * Capteurs arriÃƒÂ¨res : A0
 */
#define LFSensor_0 A1 
#define LFSensor_1 A1
#define LFSensor_2 A0
#define LFSensor_3 A0

//decode remote signal
/*
 * ils suffit de prendre un define et de le paramÃƒÂ©ter dans le smartac-lesson5 pour
 * mettre une action sur le bouton qui intÃƒÂ©resse
 */
 #define IR_ADVANCE       0x00FF18E7       //code from IR controller "Ã¢â€“Â²" button
 #define IR_BACK          0x00FF4AB5       //code from IR controller "Ã¢â€“Â¼" button
 #define IR_RIGHT         0x00FF5AA5       //code from IR controller ">" button
 #define IR_LEFT          0x00FF10EF       //code from IR controller "<" button
 #define IR_STOP          0x00FF38C7       //code from IR controller "OK" button
 #define IR_turnsmallleft 0x00FFB04F       //code from IR controller "#" button
 #define IR_one           0x00FFA25D       //code from IR controller "1" button
 #define IR_deux          0x00FF629D       //code from IR controller "2" button
 #define IR_trois         0x00FFE21D       //code from IR controller "3" button
 #define IR_four          0x00FF22DD       //code from IR controller "4" button
 #define IR_five          0x00FF02FD       //code from IR controller "5" button
 #define IR_six           0x00FFC23D       //code from IR controller "6" button
 #define IR_seven         0x00FFE01F       //code from IR controller "7" button
 #define IR_huit          0x00FFA857       //code from IR controller "8" button
 #define IR_nine          0x00FF906F       //code from IR controller "9" button
 #define IR_zero          0x00FF9867       //code from IR controller "0" button
 #define IR_asterix       0x00FF6897       //code from IR controller "*" button
//******End********

#define SERVO_PIN     9  //servo connect to D9

#define Echo_PIN   12 // Ultrasonic Echo pin connect to D11
#define Trig_PIN   11  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN     A2  //buzzer connect to D13

int sensor[5];
#define AD_SPEED1   150  //avoidance motor speed
#define BACK_SPEED1  100     //back speed
#define BACK_SPEED2  150     //back speed
#define TRACK_SPEED   150  //line follow motor speed

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
int distancelimit = 40; //distance limit for obstacles in front           //modification de la distance limite
int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)

const int turntime = 600; //Time the robot spends turning (miliseconds) 800
const int backtime = 400; //Time the robot spends turning (miliseconds) 600
int distance;
int numcycles = 0;

int thereis;
bool flag1=false;
bool stopFlag = true;
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;

#define MAX_PACKETSIZE 32    //Serial receive buffer
char buffUART[MAX_PACKETSIZE];
unsigned int buffUARTIndex = 0;
unsigned long preUARTTick = 0;

enum DS
{
  MANUAL_DRIVE,
  AUTO_DRIVE_LF, // mode combat
  AUTO_DRIVE_UO  //ultrasonic obstruction
}Drive_Status=MANUAL_DRIVE;

enum DN
{ 
  GO_ADVANCE, 
  GO_LEFT, 
  GO_RIGHT,
  GO_BACK,
  STOP_STOP,
  TIR,
  MODE_COMBAT,
  DEF
}Drive_Num=DEF;



