#include <MenuSystem.h>
#include <LiquidCrystal.h>  
#include <SoftwareSerial.h>
#include <Servo.h>

#define MOTOR_UL_CW 22
#define MOTOR_UL_CCW 23
#define MOTOR_UL_PWM 2

#define MOTOR_UR_CW 24
#define MOTOR_UR_CCW 25
#define MOTOR_UR_PWM 3

#define MOTOR_DR_CW 26
#define MOTOR_DR_CCW 27
#define MOTOR_DR_PWM 4

#define MOTOR_DL_CW 28
#define MOTOR_DL_CCW 29
#define MOTOR_DL_PWM 5

#define MOTOR_ARM0 6
#define MOTOR_ARM1 7
#define MOTOR_ARM2 8
#define MOTOR_ARM3 9
#define MOTOR_ARM4 10
 
 int motorArm0, motorArm1, motorArm2, motorArm3, motorArm4;
 Servo servoArm0, servoArm1, servoArm2, servoArm3, servoArm4;
 
#define PI 3.14159265359

#define PACKET_SIZE 6
 char packetBuffer[PACKET_SIZE];
 typedef struct PACKET {
   int8_t leftx;
   int8_t lefty;
   int8_t rightx;
   int8_t righty;
   uint16_t flags;
 };

 PACKET* packet;
 bool activeMission = false;
 
 MenuSystem menuSystem;
 Menu mainMenu("Robomoofers Menu");
 Menu menuMission1("Mission 1");
 MenuItem menuMission1_Start("Start");
 Menu menuMission2("Mission 2");
 MenuItem menuMission2_Scan("Scanning");
 MenuItem menuMission2_Solve("Shortest Path");
 Menu menuMission3("Mission 3");
 MenuItem menuMission3_Start("Start");
 Menu menuTests("Unit Tests");
 MenuItem menuTests_1("Test1");
 
 LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
 
#define NOPT_PATTERNS 6 
 String path;
 String optPatterns[NOPT_PATTERNS][2] = {
                               {  "LBR", "B"},
                               {  "LBS", "R"},
                               {  "RBL", "B"},
                               {  "SBL", "R"},
                               {  "SBS", "B"},
                               {  "LBL", "S"} 
                          };
 
 void setupMenu();
 void displayMenu();
 void controllerHandler();
 void lineTracking();
 void setupLineTracking();
 
 void optimizePath();
 void updatePath(char dir);
 
 void onMission1_StartSelected(MenuItem* item);
 void onMission2_ScanSelected(MenuItem* item);
 void onMission2_SolveSelected(MenuItem* item);
 void onMission3_StartSelected(MenuItem* item);
 void onTests_1(MenuItem* item);
 
 void moveBase(int x, int y);
 void moveArm(int x, int y);
 int moveDegree, tmpDegree, tmpPWM;
 double moveRadian, radianOffset = 0.0;
 
 bool trig0Pressed, trig1Pressed, trig2Pressed, trig3Pressed;
 bool buttonupPressed, buttondownPressed;

 void setup() {
   Serial.begin(9600);
   Serial1.begin(9600);
   lcd.begin(16, 2);
   
   pinMode(MOTOR_UR_CW, OUTPUT);
   pinMode(MOTOR_UR_CCW, OUTPUT);
   pinMode(MOTOR_UR_PWM, OUTPUT);
   pinMode(MOTOR_UL_CW, OUTPUT);
   pinMode(MOTOR_UL_CCW, OUTPUT);
   pinMode(MOTOR_UL_PWM, OUTPUT);
   pinMode(MOTOR_DL_CW, OUTPUT);
   pinMode(MOTOR_DL_CCW, OUTPUT);
   pinMode(MOTOR_DL_PWM, OUTPUT);
   pinMode(MOTOR_DR_CW, OUTPUT);
   pinMode(MOTOR_DR_CCW, OUTPUT);
   pinMode(MOTOR_DR_PWM, OUTPUT);

   analogWrite(MOTOR_UR_PWM, 0);
   analogWrite(MOTOR_UL_PWM, 0);
   analogWrite(MOTOR_DR_PWM, 0);
   analogWrite(MOTOR_DL_PWM, 0);

   servoArm0.attach(MOTOR_ARM0);
   servoArm1.attach(MOTOR_ARM1);
   servoArm2.attach(MOTOR_ARM2);
   servoArm3.attach(MOTOR_ARM3);
   servoArm4.attach(MOTOR_ARM4);

   motorArm0 = motorArm1 = motorArm2 = motorArm3 = motorArm4 = 0;
   
   //setupMenu();
   //displayMenu();
 }
 
 void loop() {
   //trig1Pressed = true;
   //moveBase(100, 0);
   controllerHandler();
   delay(10);  
 }
 
 int rad2Degree(double rad) {
   tmpDegree = (rad * 4068) / 71;
   return tmpDegree<0 ? 360+tmpDegree : tmpDegree;
 }
 
 int relativeDegree(int degree) {
   return degree;
 }
 
 double relativeRadian(double radian) {
   return radian - (3*PI/4) + radianOffset;
 }
 
#define MAX(a,b) a>b?a:b
 int tmpInt[2];
 void normalize(int* motors)
 {
   tmpInt[0] = MAX(MAX(abs(motors[0]), abs(motors[1])), MAX(abs(motors[2]), abs(motors[3])));
   tmpInt[0] = 255/tmpInt[0];
   for(tmpInt[1]=0; tmpInt[1]<4; tmpInt[1]++)
     motors[tmpInt[1]]*= tmpInt[0];
 }
 
 int baseMotorPWM[4];
 void moveBase(int x, int y) {
   if (!x && !y) {
     digitalWrite(MOTOR_UR_CW, LOW);
     digitalWrite(MOTOR_UR_CCW, LOW);
     analogWrite(MOTOR_UR_PWM, 0);
     digitalWrite(MOTOR_UL_CW, LOW);
     digitalWrite(MOTOR_UL_CCW, LOW);
     analogWrite(MOTOR_UL_PWM, 0);
     digitalWrite(MOTOR_DR_CW, LOW);
     digitalWrite(MOTOR_DR_CCW, LOW);
     analogWrite(MOTOR_DR_PWM, 0);
     digitalWrite(MOTOR_DL_CW, LOW);
     digitalWrite(MOTOR_DL_CCW, LOW);
     analogWrite(MOTOR_DL_PWM, 0);
     return;
   }
   
   if (!trig1Pressed) {
     moveRadian = atan2(-y, x);
     moveRadian = relativeRadian(moveRadian);
     
     baseMotorPWM[0] = (int)(cos(moveRadian) * 255);
     //analogWrite(MOTOR_UR_PWM, abs(baseMotorPWM[0]));
     Serial.println(baseMotorPWM[0]);
     
     if (baseMotorPWM[0] < 0) {
       digitalWrite(MOTOR_UR_CW, LOW);
       digitalWrite(MOTOR_UR_CCW, HIGH);
     } else {
       digitalWrite(MOTOR_UR_CCW, LOW);
       digitalWrite(MOTOR_UR_CW, HIGH);
     }
          
     baseMotorPWM[1] = (int)(sin(moveRadian) * 255);
     //analogWrite(MOTOR_UL_PWM, abs(baseMotorPWM[1]));
     Serial.println(baseMotorPWM[1]);
     
     if (baseMotorPWM[1] < 0) {
       digitalWrite(MOTOR_UL_CW, LOW);
       digitalWrite(MOTOR_UL_CCW, HIGH);
     } else {
       digitalWrite(MOTOR_UL_CCW, LOW);
       digitalWrite(MOTOR_UL_CW, HIGH);
     }
     
     baseMotorPWM[2] = (int)(- cos(moveRadian) * 255);
     //analogWrite(MOTOR_DL_PWM, abs(baseMotorPWM[2]));
     Serial.println(baseMotorPWM[2]);
     
     if (baseMotorPWM[2] < 0) {
       digitalWrite(MOTOR_DL_CW, LOW);
       digitalWrite(MOTOR_DL_CCW, HIGH);
     } else {
       digitalWrite(MOTOR_DL_CCW, LOW);
       digitalWrite(MOTOR_DL_CW, HIGH);
     }
     
     baseMotorPWM[3] = (int)(- sin(moveRadian) * 255);
     //analogWrite(MOTOR_DR_PWM, abs(baseMotorPWM[3]));
     Serial.println(baseMotorPWM[3]);
     
     if (baseMotorPWM[3] < 0) {
       digitalWrite(MOTOR_DR_CW, LOW);
       digitalWrite(MOTOR_DR_CCW, HIGH);
     } else {
       digitalWrite(MOTOR_DR_CCW, LOW);
       digitalWrite(MOTOR_DR_CW, HIGH);
     }
     
     normalize(baseMotorPWM);
     analogWrite(MOTOR_UR_PWM, abs(baseMotorPWM[0]));
     analogWrite(MOTOR_UL_PWM, abs(baseMotorPWM[1]));
     analogWrite(MOTOR_DL_PWM, abs(baseMotorPWM[2]));
     analogWrite(MOTOR_DR_PWM, abs(baseMotorPWM[3]));
     
     
   } else {
     if (x > 0) {
       tmpPWM = x*255/100;
       digitalWrite(MOTOR_UR_CW, LOW);
       digitalWrite(MOTOR_UR_CCW, HIGH);
       analogWrite(MOTOR_UR_PWM, tmpPWM);
       digitalWrite(MOTOR_UL_CW, LOW);
       digitalWrite(MOTOR_UL_CCW, HIGH);
       analogWrite(MOTOR_UL_PWM, tmpPWM);
       digitalWrite(MOTOR_DR_CW, LOW);
       digitalWrite(MOTOR_DR_CCW, HIGH);
       analogWrite(MOTOR_DR_PWM, tmpPWM);
       digitalWrite(MOTOR_DL_CW, LOW);
       digitalWrite(MOTOR_DL_CCW, HIGH);
       analogWrite(MOTOR_DL_PWM, tmpPWM);
       Serial.println(tmpPWM);
     } else {
       tmpPWM = -x*255/100;
       digitalWrite(MOTOR_UR_CCW, LOW);
       digitalWrite(MOTOR_UR_CW, HIGH);
       analogWrite(MOTOR_UR_PWM, tmpPWM);
       digitalWrite(MOTOR_UL_CCW, LOW);
       digitalWrite(MOTOR_UL_CW, HIGH);
       analogWrite(MOTOR_UL_PWM, tmpPWM);
       digitalWrite(MOTOR_DR_CCW, LOW);
       digitalWrite(MOTOR_DR_CW, HIGH);
       analogWrite(MOTOR_DR_PWM, tmpPWM);
       digitalWrite(MOTOR_DL_CCW, LOW);
       digitalWrite(MOTOR_DL_CW, HIGH);
       analogWrite(MOTOR_DL_PWM, tmpPWM);
       Serial.println(tmpPWM);
     }     
   }
 }

#define MOTOR_ARM_MOVEMENT 1

 void moveArm(int x, int y) {
   //Serial.println(x);
   //Serial.println(y);
   if (!trig0Pressed) {
     if (x > 0)
       motorArm0 = motorArm0+MOTOR_ARM_MOVEMENT>360 ? 360: motorArm0+MOTOR_ARM_MOVEMENT;
     else if (x < 0)
       motorArm0 = motorArm0-MOTOR_ARM_MOVEMENT<0 ? 0: motorArm0-MOTOR_ARM_MOVEMENT;
     servoArm0.write(motorArm0);
     
     if (y < 0)
       motorArm1 = motorArm1+MOTOR_ARM_MOVEMENT>180 ? 180: motorArm1+MOTOR_ARM_MOVEMENT;
     else if (y > 0)
       motorArm1 = motorArm1-MOTOR_ARM_MOVEMENT<0 ? 0: motorArm1-MOTOR_ARM_MOVEMENT;
     servoArm1.write(motorArm1);
   }
   else {
     if (buttonupPressed)
       motorArm2 = motorArm2+MOTOR_ARM_MOVEMENT>180 ? 180: motorArm2+MOTOR_ARM_MOVEMENT;
     else if (buttondownPressed)
       motorArm2 = motorArm2-MOTOR_ARM_MOVEMENT<0 ? 0: motorArm2-MOTOR_ARM_MOVEMENT;
     servoArm2.write(motorArm2); 
 
     if (y > 0)
       motorArm3 = motorArm3+MOTOR_ARM_MOVEMENT>180 ? 180: motorArm3+MOTOR_ARM_MOVEMENT;
     else if (y < 0)
       motorArm3 = motorArm3-MOTOR_ARM_MOVEMENT<0 ? 0: motorArm3-MOTOR_ARM_MOVEMENT;
     Serial.println(motorArm3);
     servoArm3.write(motorArm3);    

     if (x > 0)
       motorArm4 = motorArm4+MOTOR_ARM_MOVEMENT>180 ? 180: motorArm4+MOTOR_ARM_MOVEMENT;
     else if (x < 0)
       motorArm4 = motorArm4-MOTOR_ARM_MOVEMENT<0 ? 0: motorArm4-MOTOR_ARM_MOVEMENT;
     servoArm4.write(motorArm4);     
   }
 }
 
 void lineTracking() {
   
 }
 
 void controllerHandler() {
   if (Serial1.available() >= PACKET_SIZE) {
     Serial1.readBytes(packetBuffer, PACKET_SIZE);
     packet = (PACKET*)packetBuffer;
     
     trig0Pressed = false;
     trig1Pressed = false;
     trig2Pressed = false;
     trig3Pressed = false;
     buttonupPressed = false;
     buttondownPressed = false;
     
     // trig0
     if (packet->flags & (1 << 0)) {
       trig0Pressed = true;
     }
     // trig1
     if (packet->flags & (1 << 1)) {
       trig1Pressed = true;
     }
     // trig2
     if (packet->flags & (1 << 2)) {
       trig2Pressed = true;
     }
     // trig3
     if (packet->flags & (1 << 3)) {
       trig3Pressed = true;
     }  
     // buttonup
     if (packet->flags & (1 << 4)) {
       buttonupPressed = true;
       //Serial.println("up");
       //menuSystem.prev();
       //displayMenu();
     }     
     // buttondown
     if (packet->flags & (1 << 5)) {
       buttondownPressed = true;
       //Serial.println("down");
       //menuSystem.next();
       //displayMenu();
     }     
     // buttonleft
     if (packet->flags & (1 << 6)) {
       //Serial.println("left");
       //menuSystem.back();
       //displayMenu();
     }     
     // buttonright
     if (packet->flags & (1 << 7)) {
       //Serial.println("right");
       //menuSystem.select();
       //displayMenu();
     }     
     // triangle
     if (packet->flags & (1 << 8)) {
     }     
     // circle
     if (packet->flags & (1 << 9)) {
     }     
     // cross
     if (packet->flags & (1 << 10)) {
     }     
     // square
     if (packet->flags & (1 << 11)) {
     }     
     // select
     if (packet->flags & (1 << 12)) {
     }     
     // start
     if (packet->flags & (1 << 13)) {
     }     
     // ps
     if (packet->flags & (1 << 14)) {
     }
          
     // left analog
     moveArm(packet->leftx, packet->lefty);
     
     // right analog
     moveBase(packet->rightx, packet->righty);
     
   }
 }
 
 void displayMenu() {
   lcd.clear();
   lcd.setCursor(0,0);
   
   Menu const* cp_menu = menuSystem.get_current_menu();
   lcd.print(cp_menu->get_name());
   lcd.setCursor(0,1);
   lcd.print(cp_menu->get_selected()->get_name());
   delay(200);
 }
 
 void setupMenu() {
   mainMenu.add_menu(&menuMission1);
   menuMission1.add_item(&menuMission1_Start, onMission1_StartSelected);
   mainMenu.add_menu(&menuMission2);
   menuMission2.add_item(&menuMission2_Scan, onMission2_ScanSelected);
   menuMission2.add_item(&menuMission2_Solve, onMission2_SolveSelected);
   mainMenu.add_menu(&menuMission3);
   menuMission3.add_item(&menuMission3_Start, onMission3_StartSelected);
   mainMenu.add_menu(&menuTests);
   menuTests.add_item(&menuTests_1, onTests_1);
   menuSystem.set_root_menu(&mainMenu);
 }
 
 void onTests_1(MenuItem* item) {};
 void onMission1_StartSelected(MenuItem* item) {};
 void onMission2_ScanSelected(MenuItem* item) {};
 void onMission2_SolveSelected(MenuItem* item) {};
 void onMission3_StartSelected(MenuItem* item) {};
 
 void updatePath(char dir) {
   path += dir;
   if (path.length() > 2 && path.charAt(path.length() - 2) == 'B')
     optimizePath();
 }
 
 void optimizePath() {
   for (int i=0; i<NOPT_PATTERNS; i++)
     if (path.endsWith(optPatterns[i][0])) {
       path.replace(optPatterns[i][0], optPatterns[i][1]);
       break;
     }
 }
