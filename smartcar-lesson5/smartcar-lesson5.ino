#include <Servo.h>
#include "configuration.h"
Servo head;

#include <TM1637Display.h> 
#define CLK 10 //can be any digital pin 
#define DIO 3 //can be any digital pin
TM1637Display display(CLK, DIO);
int segA = 0b00000001;
int segB = 0b00000010;
int segC = 0b00000100;
int segD = 0b00001000;
int segE = 0b00010000;
int segF = 0b00100000;

int Score_Joueur_1;
int Score_Joueur_2;

bool wait = true;
uint8_t data[] = {0x0, 0x0, 0x0, 0x0};

#include "IRremote.h"
IRrecv IR(IR_PIN); //   IRrecv object  IR get code from IR remoter
// IRsend irsend(IR_em); //IRsend object IR 
decode_results IRresults;  

/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
}
void go_Left()  //Turn left
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
}
void go_Right()  //Turn right
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
}
void go_Back()  //Reverse
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
}
void stop_Stop()    //Stop
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,LOW);
}

/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}
void buzz_ON()   //open buzzer
{
  digitalWrite(BUZZ_PIN, LOW);
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
}

void alarm(){
   buzz_ON();
   delay(100);
   buzz_OFF();
}

//WiFi / Bluetooth through the serial control
void do_Uart_Tick()
{

  char Uart_Date=0;
  if(Serial.available()) 
  {
    size_t len = Serial.available();
    uint8_t sbuf[len + 1];
    sbuf[len] = 0x00;
    Serial.readBytes(sbuf, len);
    //parseUartPackage((char*)sbuf);
    memcpy(buffUART + buffUARTIndex, sbuf, len);//ensure that the serial port can read the entire frame of data
    buffUARTIndex += len;
    preUARTTick = millis();
    if(buffUARTIndex >= MAX_PACKETSIZE - 1) 
    {
      buffUARTIndex = MAX_PACKETSIZE - 2;
      preUARTTick = preUARTTick - 200;
    }
  }
  
  if(buffUARTIndex > 0 && (millis() - preUARTTick >= 100))//APP send flag to modify the obstacle avoidance parameters
  { //data ready
    buffUART[buffUARTIndex] = 0x00;
    
    if(buffUART[0]=='C') 
    {
      Serial.println("You have modified the parameters!");//indicates that the obstacle avoidance distance parameter has been modified
      sscanf(buffUART,"CMD%d,%d,%d",&distancelimit,&sidedistancelimit,&turntime);
      // Serial.println(distancelimit);
      // Serial.println(sidedistancelimit);
      // Serial.println(turntime);
    }
    else  Uart_Date=buffUART[0];
    buffUARTIndex = 0;
  }

  switch (Uart_Date)    //serial control instructions
  {

    case '7': tir(); break;
    case '2':Drive_Status=MANUAL_DRIVE; Drive_Num=GO_ADVANCE;Serial.println("forward"); break;
    case '4':Drive_Status=MANUAL_DRIVE; Drive_Num=GO_LEFT; Serial.println("turn left");break;
    case '6':Drive_Status=MANUAL_DRIVE; Drive_Num=GO_RIGHT; Serial.println("turn right");break;
    case '8':Drive_Status=MANUAL_DRIVE; Drive_Num=GO_BACK; Serial.println("go back");break;
    case '5':Drive_Status=MANUAL_DRIVE; Drive_Num=STOP_STOP;buzz_OFF();Serial.println("stop");break;
    case '3':Drive_Status=AUTO_DRIVE_UO; Serial.println("avoid obstacles...");break;
    case '1':Drive_Status=AUTO_DRIVE_LF; Serial.println("line follow...");break;
    default:break;
  }
}

void Depassement(){
  sensor[0]= digitalRead(LFSensor_0);
  sensor[1]= digitalRead(LFSensor_1);
  sensor[2]= digitalRead(LFSensor_2);
  sensor[3]= digitalRead(LFSensor_3);

  if(sensor[0] == 1 or sensor[1] == 1){
    stop_Stop();
    delay(5000); /* 5 secondes */

    go_Back();
    set_Motorspeed(255,255);
    delay(1000);
    stop_Stop();
  }
  if(sensor[2] == 1 or sensor[3] == 1){
    stop_Stop();
    delay(5000); /* 5 secondes */

    go_Advance();
    set_Motorspeed(255,255);
    delay(1000);
    stop_Stop();
  }
}

//************************  Gestion Ir *********************************
/*
 * Fonction qui commande la tÃƒÂ©lÃƒÂ©commande
 * pour chaque dÃƒÂ©placement ou touche,
 * elle rappelle le dÃƒÂ©passement 
 */
void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
    if(IRresults.value==IR_ADVANCE)
    {
      Drive_Num=GO_ADVANCE;
      Depassement();
    }
    else if(IRresults.value==IR_RIGHT)
    {
       Drive_Num=GO_RIGHT;
       Depassement();
    }
    else if(IRresults.value==IR_LEFT)
    {
       Drive_Num=GO_LEFT;
       Depassement();
    }
    else if(IRresults.value==IR_BACK)
    {
        Drive_Num=GO_BACK;
        Depassement();
    }
    else if(IRresults.value==IR_STOP)
    {
      //touche "ok" pour stop stop
        Drive_Num=STOP_STOP;
    }
    else if(IRresults.value==IR_turnsmallleft)
    {
      //touche "#" pour demarrer la detection des obstacles automatique
      Drive_Status=AUTO_DRIVE_UO;
      Depassement();
    }
    else if(IRresults.value==IR_one)
    {
      //touche "1" pour rÃƒÂ©nitialiser le score
      wait = false;
      display.setBrightness(0x0f);
      Display_Start();
    }else if(IRresults.value==IR_deux)
    {
      //touche "2" pour activer l'emetteur
    }
    IRresults.value = 0;
    IR.resume();
  }
}

/*
 * Fonction de mise ÃƒÂ  1 l'emetteur IR
 */
void IREmitterOn(){
  // irsend.mark(0); 
  pause(10); 
}
 /*
  * Fonction de mise ÃƒÂ  0 l'ÃƒÂ©metteur IR
  */
void IREmitterOff(){
  // irsend.space(0); 
  pause(60);
}
void switchOffOnIREmitter() { 
  IREmitterOff(); 
  IREmitterOn(); 
}
/*
 * cette fonction remplace le delay , car delay ne 
 * fonctionne pas dans les rappels d'interruption0
 */
  void pause(int ms)
  {
    for (int i=0; i< ms; i++)
    {
      delayMicroseconds(1000);
    }
  }

/*
 * Fonction pour tirer
 */
void tir()
{
  Serial.println("TIIIIIR");
}
/**************************************************************************/

//car motor control
void do_Drive_Tick()
{
  if(Drive_Status == MANUAL_DRIVE)
  {
    switch (Drive_Num) 
    {
      case GO_ADVANCE:
          go_Advance();
          set_Motorspeed(255,255);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          Depassement();
          break;
      case GO_LEFT: 
          go_Left();
          set_Motorspeed(255,255);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          Depassement();
          break;
      case GO_RIGHT:  
          go_Right();
          set_Motorspeed(255,255);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          Depassement();
          break;
      case GO_BACK: 
          go_Back();
          set_Motorspeed(255,255);
          JogFlag = true;
          JogTimeCnt = 1;
          JogTime=millis();
          Depassement();
          break;
      case STOP_STOP: 
          stop_Stop();
          JogTime = 0;
          break;
      default:
          break;
    }
    Drive_Num=DEF;
    //keep the car running for 100ms
    if(millis()-JogTime>=100)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        stop_Stop();
      }
    }
  }
  else if(Drive_Status==AUTO_DRIVE_LF)
  {
    /* auto_tarcking(); */
  }
  else if(Drive_Status==AUTO_DRIVE_UO)
  {
   auto_avoidance();
  }
}

/**************car control**************/
void do_Drive_Tick2()
{
    switch (Drive_Num) 
    {
      case GO_ADVANCE:go_Advance();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_ADVANCE code is detected, then go advance
      case GO_LEFT: go_Left();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_LEFT code is detected, then turn left
      case GO_RIGHT:  go_Right();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_RIGHT code is detected, then turn right
      case GO_BACK: go_Back();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_BACK code is detected, then backward
      case STOP_STOP: stop_Stop();JogTime = 0;break;//stop
      default:break;
    }
    Drive_Num=DEF;
   //keep current moving mode for  200 millis seconds
    if(millis()-JogTime>=200)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        stop_Stop();
      }
    }
}

/******************** Gestion de l'afficheur ********************/
void Display_on()/* Afficher tous */
{
  data[0] = 0xff;
  data[1] = 0xff;
  data[2] = 0xff;
  data[3] = 0xff;
  display.setSegments(data);
}
void Display_off() /* eteindre tous */  
{
  data[0] = 0x0;
  data[1] = 0x0;
  data[2] = 0x0;
  data[3] = 0x0;
  
  display.setSegments(data);
}
void Display_Center()
{
  uint8_t segto;
  segto = 0x80;
  display.setSegments(&segto, 1, 1);
}

int time2 = 0;
void Display_Wait(int timeloop)
{
  if(wait)
  {
    if(timeloop != time2){
      time2 = timeloop;
      switch (data[0])
      {
        case 0x0:
          data[0] = segA;
          data[1] = segA;
          data[2] = segA;
          data[3] = segA;
          break;
        case 0b00000001:
          data[0] = segB;
          data[1] = segB;
          data[2] = segB;
          data[3] = segB;
          break;
        case 0b00000010:
          data[0] = segC;
          data[1] = segC;
          data[2] = segC;
          data[3] = segC;
          break;
        case 0b00000100:
          data[0] = segD;
          data[1] = segD;
          data[2] = segD;
          data[3] = segD;
          break;
        case 0b00001000:
          data[0] = segE;
          data[1] = segE;
          data[2] = segE;
          data[3] = segE;
          break;
        case 0b00010000:
          data[0] = segF;
          data[1] = segF;
          data[2] = segF;
          data[3] = segF;
          break;
        case 0b00100000:
          data[0] = segA;
          data[1] = segA;
          data[2] = segA;
          data[3] = segA;
          break;
      }
      display.setSegments(data);
    }
    
    
  }
}

void Display_Start()
{
  Display_off();
  Score_Joueur_1 = 0;
  Score_Joueur_2 = 0;

  for (int i=0; i< 3; i++)
  {
    Display_on();
    delay(500);
    Display_off();
    delay(500);
  }
  
  display.showNumberDec(Score_Joueur_1,false);
  Display_Center();
  display.showNumberDec(Score_Joueur_2,false,1,0);
}

void Add_Joueur_1()
{
  Display_off();
  Score_Joueur_1 = Score_Joueur_1 + 1;
  display.showNumberDec(Score_Joueur_2,false,1,0);
  Display_Center();
  display.showNumberDec(Score_Joueur_1,false);
}

/*
 * Ici  je considÃƒÂ¨re que c'est la fonction qui incrÃƒÂ©mente le 
 * score du joueur 2
 * je rajoute alarm(), car si touchÃƒÂ© , il faut ÃƒÂ©mettre un bip 
 * avant de rÃƒÂ©nitialiser affichage
 */
void Add_Joueur_2()
{
  alarm();
  Display_off();
  Score_Joueur_2 = Score_Joueur_2 + 1;
  display.showNumberDec(Score_Joueur_2,false,1,0);
  Display_Center();
  display.showNumberDec(Score_Joueur_1,false);
}


void setup()
{
  pinMode(dir1PinL, OUTPUT); 
  pinMode(dir2PinL, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();
  
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 

  //ir remote
  Serial.begin(9600);
  IR.enableIRIn(); 

  // irsend.enableIROut(38);
  IREmitterOn();
  //end of ir remote

  pinMode(IR_em, OUTPUT);
  
  pinMode(BUZZ_PIN, OUTPUT);
  buzz_OFF();  

  /*
  pinMode(LFSensor_0,INPUT);
  pinMode(LFSensor_1,INPUT);
  pinMode(LFSensor_2,INPUT);
  pinMode(LFSensor_3,INPUT);
  pinMode(LFSensor_4,INPUT);
  */
  
  Serial.begin(9600);//In order to fit the Bluetooth module's default baud rate, only 9600
  digitalWrite(Trig_PIN,LOW);
  head.attach(SERVO_PIN); //servo
  head.write(90);
  
  
  display.setBrightness(0x0f);
}

void loop()
{
  Display_Wait(millis()/100);
  do_Uart_Tick();
  do_IR_Tick(); //ir remote
  do_Drive_Tick();
  do_Drive_Tick2();
}




