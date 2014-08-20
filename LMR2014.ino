 #include <MenuSystem.h>
 #include <LiquidCrystal.h>
 #include <Thread.h>
  
 #define MISSION_1
 
 #define PACKET_SIZE 18
 char packetBuffer[PACKET_SIZE];
 typedef struct PACKET {
   uint32_t leftx;
   uint32_t lefty;
   uint32_t rightx;
   uint32_t righty;
   uint16_t flags;
 };
 
 PACKET* packet;
 
 // Menu variables
 MenuSystem menuSystem;
 Menu mainMenu("Robomoofers Menu");
 #ifdef MISSION_1
 Menu menuMission1("Mission 1");
 MenuItem menuMission1_Scan("Scanning");
 MenuItem menuMission1_Solve("Shortest Path");
 
 #elif defined(MISSION_2)
 Menu menuMission2("Mission 2");
 MenuItem menuMission2_Start("Start");
 
 #elif defined(MISSION_3)
 Menu menuMission3("Mission 3");
 MenuItem menuMission3_Start("Start");
 #endif
 
 /*
 The LCD circuit:
  * LCD RS pin to digital pin 8
  * LCD Enable pin to digital pin 9
  * LCD D4 pin to digital pin 4
  * LCD D5 pin to digital pin 5
  * LCD D6 pin to digital pin 6
  * LCD D7 pin to digital pin 7
  * LCD R/W pin to ground
  */
 // initialize the library with the numbers of the interface pins
 LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
 
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
 void serialHandler();
 void lineTracking();
 void setupLineTracking();
 
 void optimizePath();
 void updatePath(char dir);

 void onMission1_ScanSelected(MenuItem* item);
 void onMission1_SolveSelected(MenuItem* item);
 void onMission2_StartSelected(MenuItem* item);
 void onMission3_StartSelected(MenuItem* item);
 
 Thread controllerThread = Thread();
 
 void setup() {
   Serial.begin(9600);
   myThread.onRun(serialHandler);
   myThread.setInterval(100);
   
   lcd.begin(16, 2);
   
   setupMenu();
   setupLineTracking();
   displayMenu();
 }
 
 void loop() {
   if(controllerThread.shouldRun())
     controllerThread.run();
   
   displayMenu();  
 }
 
 
 void lineTracking() {
 
 }
 
 void setupLineTracking() {
   pinMode(sensorRight,INPUT);
   pinMode(sensorLeft,INPUT);
   pinMode(motorRight,OUTPUT);
   pinMode(motorLeft,INPUT);
 }
 
 void serialHandler() {
   while(1) {
     if (Serial.available() >= PACKET_SIZE) {
       Serial.readBytes(packetBuffer, PACKET_SIZE);
       packet = (PACKET*)packetBuffer;
     }
   }
 }
 
 void displayMenu() {
   lcd.clear();
   lcd.setCursor(0,0);
   
   Menu const* cp_menu = menuSystem.get_current_menu();
   lcd.print(cp_menu->get_name());
   lcd.setCursor(0,1);
   lcd.print(cp_menu->get_selected()->get_name());
 }
 
 void setupMenu() {
 #ifdef MISSION_1
   mainMenu.add_menu(&menuMission1);
   menuMission1.add_item(&menuMission1_Scan, onMission1_ScanSelected);
   menuMission1.add_item(&menuMission1_Solve, onMission1_SolveSelected);
 #elif defined(MISSION_2)
   mainMenu.add_menu(&menuMission2);
   menuMission1.add_item(&menuMission2_Start, onMission2_StartSelected);
 #elif defined(MISSION_3)
   mainMenu.add_menu(&menuMission3);
   menuMission1.add_item(&menuMission3_Start, onMission3_StartSelected);
 #endif
   menuSystem.set_root_menu(&mainMenu);
 }
 
 #ifdef MISSION_1
 void onMission1_ScanSelected(MenuItem* item) {};
 void onMission1_SolveSelected(MenuItem* item) {};
 #elif defined(MISSION_2)
 void onMission2_StartSelected(MenuItem* item) {};
 #elif defined(MISSION_3)
 void onMission3_StartSelected(MenuItem* item) {};
 #endif
 
 #ifdef MISSION_1
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
 #endif

