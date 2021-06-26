#include "ZeroBug.h"

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

#define LED PB5
#define SDA1 PB7
#define SCL1 PB6
#define PCA_OE PB8

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVOMIN  604.0 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2260.0 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Servo servo[4]; // local pwm servos

gaitEngine Hex;

byte tripodGait[120][6];
byte waveGait[120][6];

float jX = 0; // (Joystick) Walking X, Y and Rotate
float jY = 0.0;
float jR = 0.0;

long frameCounter = 0;

char inputBuffer[64]; // = "m 123.123 000.456 789.001";

float vBat = 0;

boolean IKError = false;

boolean powerup = false;
boolean powerdown = false;

boolean clawOpen = false;
boolean clawClose = false;
float clawPos = 0;

long controlTimeout;

void setup() {

  pinMode(LED, OUTPUT);
  pinMode(PCA_OE, OUTPUT);  
  digitalWrite(PCA_OE, HIGH);
  
  // tripod gait
  Hex.gaitSpeed = 1.0;
  Hex.legSpeed = 1.0;
  tripodGait[0][0]  = 1;
  tripodGait[0][2]  = 1;
  tripodGait[0][4]  = 1;
  
  tripodGait[30][1] = 1;
  tripodGait[30][3] = 1;
  tripodGait[30][5] = 1;
  
  tripodGait[60][0] = 1;
  tripodGait[60][2] = 1;
  tripodGait[60][4] = 1;
  
  tripodGait[90][1] = 1;
  tripodGait[90][3] = 1;
  tripodGait[90][5] = 1;
  
  
  // wave gait
  //Hex.gaitSpeed = 2.0;
  //Hex.legSpeed = 0.7;
  waveGait[00][0]  = 1;
  waveGait[20][4]  = 1;
  waveGait[40][2]  = 1;
  waveGait[60][5]  = 1;
  waveGait[80][1]  = 1;
  waveGait[100][3] = 1;
  
  memcpy(Hex.gSeq, tripodGait, sizeof(Hex.gSeq));
  
  Serial1.begin(115200);
  inputBuffer[0] = '\0'; //Initialize string to emtpy
  
  //Reset I2C bus
  pinMode(SDA1, OUTPUT);
  pinMode(SCL1, OUTPUT);
  digitalWrite(SDA1, HIGH);
  digitalWrite(SCL1, HIGH);
  delay(200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  //Wire.setClock(400000);
  
  enableServos(true);
}

void loop() {

  if(controlTimeout < millis()) digitalWrite(LED, millis() % 200 <= 100);
  else digitalWrite(LED, millis() % 2000 <= 1000);
  

  if(powerup){
    if(Hex.traZ > 0){
      Hex.legS[0].setto(-70, -85, 0);
      Hex.legS[1].setto(-90, 0, 0);
      Hex.legS[2].setto(-70, 85, 0);
      Hex.legS[3].setto(70, 85, 0);
      Hex.legS[4].setto(90, 0, 0);
      Hex.legS[5].setto(70, -85, 0);
      Hex.traZ -= 0.3;
    }
    else{      
      powerup = false;
    }
  }
  else if(powerdown){
    if(Hex.traZ < 26){
      Hex.legS[0].setto(-65, -70, 0);
      Hex.legS[1].setto(-80, 0, 0);
      Hex.legS[2].setto(-65, 70, 0);
      Hex.legS[3].setto(65, 70, 0);
      Hex.legS[4].setto(80, 0, 0);
      Hex.legS[5].setto(65, -70, 0);
      Hex.traZ += 0.3;
    }
    else{
     powerdown = false;
    }
  }

  clawPos = constrain(clawPos, 0, 67);
  if(clawOpen){
    if(clawPos > 0) clawPos -= 2;
    else clawOpen = false;
  }
  else if(clawClose){
    if(clawPos < 58) clawPos += 2;
    else clawClose = false;
  }

  //stop hexapod if there is no connection from Raspi
  if(controlTimeout < millis()) {
    jX = 0;
    jY = 0;
    jR = 0;
  }
    
  // Configure and run the hexapod engine  
  Hex.walkX = jX;
  Hex.walkY = jY;
  Hex.walkR = jR;
  // Check for error in previous loop
  if(IKError){
    jX = 0;
    jY = 0;
    jR = 0;
  }  
  Hex.gaitStep();
  Hex.runBodyIK();
  IKError = (Hex.runLegIK() != 0);
  
  /*
  float angleA = 0;
  float angleB = 0;
  float angleC = 0;
  float angleD = 0;
  float angleE = 0;
  float angleF = 0;
  float angleG = 0;
  float angleH = 0;
  float angleI = 0;
  
  float angleJ = 0;
  float angleK = 0;
  float angleL = 0;
  float angleM = 0;
  float angleN = 0;
  float angleO = 0;
  float angleP = 0;
  float angleQ = 0;
  float angleR = 0;
  */
  
  float angleA = constrain( Hex.legAngle[0][0]*180/PI, -65, 65);
  float angleB = constrain(-Hex.legAngle[0][1]*180/PI, -100, 100);
  float angleC = constrain( Hex.legAngle[0][2]*180/PI, -100, 100);
  float angleD = constrain( Hex.legAngle[1][0]*180/PI, -65, 65);
  float angleE = constrain(-Hex.legAngle[1][1]*180/PI, -100, 100);
  float angleF = constrain( Hex.legAngle[1][2]*180/PI, -100, 100);
  float angleG = constrain( Hex.legAngle[2][0]*180/PI, -65, 65);
  float angleH = constrain(-Hex.legAngle[2][1]*180/PI, -100, 100);
  float angleI = constrain( Hex.legAngle[2][2]*180/PI, -100, 100);
  
  float angleJ = constrain( Hex.legAngle[3][0]*180/PI, -65, 65);  
  float angleK = constrain( Hex.legAngle[3][1]*180/PI, -100, 100);
  float angleL = constrain(-Hex.legAngle[3][2]*180/PI, -100, 100);
  float angleM = constrain( Hex.legAngle[4][0]*180/PI, -65, 65);
  float angleN = constrain( Hex.legAngle[4][1]*180/PI, -100, 100);
  float angleO = constrain(-Hex.legAngle[4][2]*180/PI, -100, 100);
  float angleP = constrain( Hex.legAngle[5][0]*180/PI, -65, 65);
  float angleQ = constrain( Hex.legAngle[5][1]*180/PI, -100, 100);
  float angleR = constrain(-Hex.legAngle[5][2]*180/PI, -100, 100);
  
  float angleS = constrain(Hex.rotZ*180/PI*1.7, -65, 65);
  float angleT = constrain(clawPos, 0, 67);

  angleA += -24;
  angleB += 18;
  angleC += 3;
  
  angleD += 14;
  angleE += 2;
  angleF += 0;
  
  angleG += 50;
  angleH += -12;
  angleI += -1;

  //---
  
  angleJ += -35;
  angleK += 11;
  angleL += 6;
  
  angleM += 3;
  angleN += 12;
  angleO += -6;

  angleP += 23;
  angleQ += 0;
  angleR += 0;

  angleS += -8;
  angleT += 0;

  
  int servoVALA = map(angleA*1000, -90*1000, 90*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALB = map(angleB*1000, -100*1000, 100*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALC = map(angleC*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALD = map(angleD*1000, -90*1000, 90*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALE = map(angleE*1000, -100*1000, 100*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALF = map(angleF*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALG = map(angleG*1000, -90*1000, 90*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALH = map(angleH*1000, -100*1000, 100*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALI = map(angleI*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  
  int servoVALJ = map(angleJ*1000, -90*1000, 90*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALK = map(angleK*1000, -100*1000, 100*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALL = map(angleL*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALM = map(angleM*1000, -90*1000, 90*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALN = map(angleN*1000, -100*1000, 100*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALO = map(angleO*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALP = map(angleP*1000, -90*1000, 90*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALQ = map(angleQ*1000, -100*1000, 100*1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALR = map(angleR*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;

  int servoVALS = map(angleS*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
  int servoVALT = map(angleT*1000, -90*1000, 90 *1000, SERVOMIN*1000, SERVOMAX*1000)/1000.0;
    
  static int testVal = SERVOMIN;
  if(testVal >= SERVOMAX) testVal = SERVOMIN;

  vBat = (float(analogRead(PA3))/4069.0)*10.321; // 4.7K+10K voltage divider

  batteryCheck();
    
  if(IKError){
    Serial.println("IKError Occured");
  }  
  else {
    pwm.writeMicroseconds(0, servoVALM);
    pwm.writeMicroseconds(1, servoVALN);
    pwm.writeMicroseconds(2, servoVALO);
    
    pwm.writeMicroseconds(3, servoVALP);  
    pwm.writeMicroseconds(4, servoVALQ);
    pwm.writeMicroseconds(5, servoVALR);
    
    pwm.writeMicroseconds(6, servoVALA);
    pwm.writeMicroseconds(7, servoVALB);
    pwm.writeMicroseconds(8, servoVALC); 

    servo[0].writeMicroseconds(servoVALD);
    servo[1].writeMicroseconds(servoVALE);
    servo[2].writeMicroseconds(servoVALF);
    
    servo[3].writeMicroseconds(servoVALJ);
    pwm.writeMicroseconds(15, servoVALK);    
    pwm.writeMicroseconds(14, servoVALL);
    
    pwm.writeMicroseconds(13, servoVALG);
    pwm.writeMicroseconds(12, servoVALH);  
    pwm.writeMicroseconds(11, servoVALI);   

    pwm.writeMicroseconds(10, servoVALS);
    pwm.writeMicroseconds(9, servoVALT);   
  }
  
  while (Serial1.available()>0){
    char input = Serial1.read();
    static int s_len; // static variables default to 0    
    if ((s_len >= 64) || (input != '\n' && input != '\r')) {
      inputBuffer[s_len++] = input;
    } else {      
      char commandChar = inputBuffer[0];      
      char * strtokIndx; // this is used by strtok() as an index
      strtokIndx = strtok(inputBuffer, " ");
      strtokIndx = strtok(NULL, " ");
      if(commandChar == 'm'){ // MOVE
        jX = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        jY = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        jR = atof(strtokIndx);
      }
      else if(commandChar == 't'){ // TRANSLATE
        Hex.traX = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.traY = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.traZ += atof(strtokIndx); // vorsicht!!! rückgängig machen?
      }
      else if(commandChar == 'r'){ // ROTATE
        Hex.rotX = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.rotY = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.rotZ = atof(strtokIndx);
      }
      else if(commandChar == 'u'){ // UP
        powerup = true;
      }
      else if(commandChar == 'd'){ // DOWN
        powerdown = true;
      }
      else if(commandChar == 'a'){ // Tripod Gait
        Hex.gaitSpeed = 1.0;
        Hex.legSpeed = 1.0;
        memcpy(Hex.gSeq, tripodGait, sizeof(Hex.gSeq));
      }
      else if(commandChar == 'b'){ // Wave Gait
        Hex.gaitSpeed = 2.0;
        Hex.legSpeed = 1.3;
        memcpy(Hex.gSeq, waveGait, sizeof(Hex.gSeq));
      }
      else if(commandChar == 'c'){ // CLOSE
        clawClose = true;
      }
      else if(commandChar == 'o'){ // OPEN
        clawOpen = true;
      }
      else if(commandChar == 'g'){ // CLAW fine adjust
        clawPos += atof(strtokIndx);
      }
      else if(commandChar == 'h'){ // heartbeat
        controlTimeout = millis()+3000;
      }
      // Reset buffer
      memset(inputBuffer, 0, sizeof(inputBuffer));
      s_len = 0;      
    }
  }
  
  while (Serial.available()>0){
    char input = Serial.read();
    static int s_len; // static variables default to 0    
    if ((s_len >= 64) || (input != '\n' && input != '\r')) {
      inputBuffer[s_len++] = input;
    } else {      
      char commandChar = inputBuffer[0];      
      char * strtokIndx; // this is used by strtok() as an index
      strtokIndx = strtok(inputBuffer, " ");
      strtokIndx = strtok(NULL, " ");
      if(commandChar == 'm'){ // MOVE
        jX = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        jY = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        jR = atof(strtokIndx);
      }
      else if(commandChar == 't'){ // TRANSLATE
        Hex.traX = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.traY = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.traZ = atof(strtokIndx);
      }
      else if(commandChar == 'r'){ // ROTATE
        Hex.rotX = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.rotY = atof(strtokIndx);
        strtokIndx = strtok(NULL, " ");
        Hex.rotZ = atof(strtokIndx);
      }
      else if(commandChar == 'u'){ // UP
        powerup = true;
      }
      else if(commandChar == 'd'){ // DOWN
        powerdown = true;
      }
      else if(commandChar == 'a'){ // Tripod Gait
        Hex.gaitSpeed = 1.0;
        Hex.legSpeed = 1.0;
        memcpy(Hex.gSeq, tripodGait, sizeof(Hex.gSeq));
      }
      else if(commandChar == 'b'){ // Wave Gait
        Hex.gaitSpeed = 2.0;
        Hex.legSpeed = 1.3;
        memcpy(Hex.gSeq, waveGait, sizeof(Hex.gSeq));
      }
      else if(commandChar == 'c'){ // CLOSE
        clawClose = true;
      }
      else if(commandChar == 'o'){ // OPEN
        clawOpen = true;
      }
      else if(commandChar == 'g'){ // CLAW fine adjust
        clawPos += atof(strtokIndx);
      }
      // Reset buffer
      memset(inputBuffer, 0, sizeof(inputBuffer));
      s_len = 0;      
    }
  }
  
  telemetry();

  debugOutput();
  
  //delay(10);
}

void debugOutput(){
  Serial.print("DBG");
  Serial.print(" ");
  Serial.print(jX);
  Serial.print(" ");
  Serial.print(jY);
  Serial.print(" ");
  Serial.print(jR);
  for(int i=0; i < 6; i++){
    Serial.print(" ");
    Serial.print(Hex.legAngle[i][0], 2);
    Serial.print(" ");
    Serial.print(Hex.legAngle[i][1], 2);
    Serial.print(" ");
    Serial.print(Hex.legAngle[i][2], 2);
  }
  Serial.print(" ");
  frameCounter = micros() - frameCounter;
  Serial.print(frameCounter/1000.0);
  Serial.print("ms ");  
  Serial.print(vBat);
  Serial.println("V");
  frameCounter = micros();
    
}

void telemetry(){
  static int cycleTimer = 0;
  if(cycleTimer++ > 40){
    Serial1.print("v ");
    Serial1.print(vBat);
    Serial1.print("\n");
    cycleTimer = 0;
  }  
}

// Disable servos if voltage is too low
void batteryCheck(){
  static int batteryTimer = 0;
  if(vBat < 5){
    if(batteryTimer > 50){
      enableServos(false);
    }
    batteryTimer++;
  }
  else if(vBat > 6){
    enableServos(true);
    batteryTimer = 0;
  }
}

void enableServos(boolean servOn){
  static boolean servosEnabled = false;
  if(!servosEnabled && servOn){
    digitalWrite(PCA_OE, LOW);
    servo[0].attach(PB1, SERVOMIN, SERVOMAX);
    servo[1].attach(PB0, SERVOMIN, SERVOMAX);
    servo[2].attach(PA7, SERVOMIN, SERVOMAX);
    servo[3].attach(PA6, SERVOMIN, SERVOMAX);
    servosEnabled = true;
  }
  else if(servosEnabled && !servOn){
    digitalWrite(PCA_OE, HIGH);
    servo[0].detach();
    servo[1].detach();
    servo[2].detach();
    servo[3].detach();
    servosEnabled = false;
  }
}
