
class gaitEngine {
  
  //public:
  float gaitStepN = 0;
  byte gSeq[][] = new byte[120][6];
  byte[] legMoving = new byte[6];
  float gaitSpeed = 1.0; // control gait speed by adjusting gaitStepN progression
  float legSpeed = 1.0; // control gait speed by adjusting leg lift function
  float stepHeight = 20;
  float targetDeadZone = 3; // allow off center leg position
  float walkX = 0; // inputs for walking in X,Y and rotation
  float walkY = 0;
  float walkR = 0;
  float traX, traY, traZ; // current body translation
  float rotX, rotY, rotZ; // current for body rotation
  // hexapod constants
  int coxa  = (int)11.6;
  int femur = 44;
  int tibia = 69;
  
  vector[] coxaPos = new vector[6]; // body position
  vector[] leg = new vector[6]; //current leg positions (without body translation/rotation)
  vector[] legS = new vector[6]; //defaulegT leg positions
  vector[] legT = new vector[6]; //target leg positions
  vector[] legIK = new vector[6]; //leg positions for body translation/rotation
  
  float[][] legAngle = new float[6][3];
  
  gaitEngine() { // Initialization
    
    for (int i = 0; i < 6; i++) {
      leg[i] = new vector();
      legS[i] = new vector();
      legT[i] = new vector();
      legIK[i] = new vector();
      coxaPos[i] = new vector();
    }
    leg[0].setto(-70, -85, 0);
    leg[1].setto(-90, 0, 0);
    leg[2].setto(-70, 85, 0);
    leg[3].setto(70, 85, 0);
    leg[4].setto(90, 0, 0);
    leg[5].setto(70, -85, 0);
    
    for (int i = 0; i < 6; i++) {
      legS[i].setto(leg[i]);
      legT[i].setto(leg[i]);
    }
    
    int bodyHeight = 40;
    coxaPos[0].setto(-28, -48, bodyHeight);
    coxaPos[1].setto(-35, 0, bodyHeight);
    coxaPos[2].setto(-28, 48, bodyHeight);
    coxaPos[3].setto(28, 48, bodyHeight);
    coxaPos[4].setto(35, 0, bodyHeight);
    coxaPos[5].setto(28, -48, bodyHeight);
  }
  
  void gaitStep() {
  
    walkX = constrain(walkX, -0.8, 0.8);
    walkY = constrain(walkY, -0.6, 0.6);
    walkR = constrain(walkR, -0.6, 0.6);
  
    gaitStepN += gaitSpeed;
    int gSeqLength = gSeq.length;
    if(gaitStepN >= gSeqLength) gaitStepN = 0;
    
    //calculate rotation and translation of legs over ground  
    float s = sin(-walkR/100);
    float c = cos(-walkR/100);
    
    //move legs with ground
    for(int i = 0; i < 6; i++){
      if(leg[i].z == 0){
        leg[i].x = (leg[i].x*c - leg[i].y*s) - walkX;
        leg[i].y = (leg[i].y*c + leg[i].x*s) + walkY;
        //move target within boundary to allow some movement without stepping 
        if(leg[i].z == 0 && sqrt(sq(leg[i].x-legS[i].x)+sq(leg[i].y-legS[i].y )) < targetDeadZone ){
          legT[i].x = leg[i].x;
          legT[i].y = leg[i].y;
        }
        else { // reset target when getting to far off center
          legT[i].setto(legS[i]);
        }
      }
    }
  
    for(int legN = 0; legN < 6; legN++){ // cycle through all legs
      
      // if leg's turn in sequence AND if not on target AND not already moving
      // TODO: Do not rely on rounding gaitStepN to match the sequence in gSeq
      if(gSeq[(int)gaitStepN][legN] == 1 && !leg[legN].equalegS(legT[legN]) && legMoving[legN] == 0){ // check gait sequence to see which legs should start moving
        //println("gaitStepN: " + gaitStepN + " legN: " + legN);
        leg[legN].xt = legS[legN].x;
        leg[legN].yt = legS[legN].y;
        leg[legN].xs = leg[legN].x;
        leg[legN].ys = leg[legN].y;
        legMoving[legN] = 1;
      }
      
      // if leg ist already moving
      if(legMoving[legN] == 1){
        float distanceLeft = leg[legN].move2D(legSpeed); 
        //leg[legN].z = legS[legN].z + stepHeight-stepHeight*sq(2*distanceLeft-1);//quadratic function leg lift
        leg[legN].z = legS[legN].z + stepHeight/2*(sin(2*PI*(distanceLeft-0.25))+1);//smoother sinus leg lift        
        if(leg[legN].z == 0) legMoving[legN] = 0;
      }     
      
    }    
  }
  
  // Body Translation/Rotation
  void runBodyIK(){ 

    traX = constrain(traX, -40, 40);
    traY = constrain(traY, -40, 40);
    traZ = constrain(traZ, -40, 40);
    
    rotX = constrain(rotX, -0.3, 0.3);
    rotY = constrain(rotY, -0.25, 0.25);
    rotZ = constrain(rotZ, -0.25, 0.25);
       
    for(int i = 0; i < 6; i++){
      legIK[i].x = leg[i].x*cos(rotZ)*cos(rotX)-leg[i].z*cos(rotZ)*sin(rotX)+leg[i].y*sin(rotZ)+traX;
      legIK[i].y = leg[i].x*(sin(rotY)*sin(rotX)-cos(rotY)*sin(rotZ)*cos(rotX))+leg[i].z*(cos(rotY)*sin(rotZ)*sin(rotX)+sin(rotY)*cos(rotX))+leg[i].y*cos(rotY)*cos(rotZ)+traY;
      legIK[i].z = leg[i].x*(sin(rotY)*sin(rotZ)*cos(rotX)+cos(rotY)*sin(rotX))+leg[i].z*(sin(rotY)*sin(rotZ)*sin(rotX)+cos(rotY)*cos(rotX))-leg[i].y*sin(rotY)*cos(rotZ)+traZ;
    }
  }
  
  // Full Leg IK (based on https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/)
  int runLegIK(){
    int mathError = 0;
    for(int i = 0; i < 6; i++){
      float deltaX = (legIK[i].x - coxaPos[i].x);
      float deltaY = (legIK[i].y - coxaPos[i].y);
      float deltaZ = -(legIK[i].z - coxaPos[i].z);
      
      float legLength = sqrt(sq(deltaX) + sq(deltaY));
      float HF = sqrt(sq(legLength - coxa)+sq(deltaZ));
      // Throw error if target is unreachable
      if((HF > femur+tibia) || (HF < abs(femur-tibia))) mathError = 1;        
      float AX1 = atan((legLength - coxa)/deltaZ);
      if(AX1 < 0) AX1 = PI+AX1;
      float AX2 = acos((sq(tibia)-sq(femur)-sq(HF))/(-2*femur*HF));
      legAngle[i][1] = PI/2-(AX1+AX2); // femur angle
      float BX1 = acos((sq(HF)-sq(tibia)-sq(femur))/(-2*femur*tibia));
      legAngle[i][2] = PI/2-BX1; // tibia angle
      legAngle[i][0] = atan(deltaY/deltaX); // coxa angle
      /*
      if(i == 0){
        println("legLength: " + legLength);
        println("HF: " + HF);
        println("AX1: " + AX1);
        println("AX2: " + AX2);
        println("BX1: " + BX1);
        println("angle[i][0]: " + legAngle[i][0]*(180/PI) + " angle[i][1]: " + legAngle[i][1]*(180/PI) + " angle[i][2]: " + legAngle[i][2]*(180/PI));
        println("angle[i][0]: " + legAngle[i][0] + " angle[i][1]: " + legAngle[i][1] + " angle[i][2]: " + legAngle[i][2]);
        println("deltaX: " + deltaX + " deltaY: " + deltaY + " deltaZ: " + deltaZ);
      } */
      // Rotate Coxa angles on one side of the robot
      //if(i > 2){ 
      //  legAngle[i][0] += PI;
      //}
    }
    return mathError;
  }
  
}

// derived from Oscar Liangs legged robot code base
class vector {

  //public:
  float x;
  float y;
  float z;
  
  float xs;
  float ys;
  float zs;
  
  float xt;
  float yt;
  float zt;
  int oldmillis = 0;

  //public:
  vector() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }

  void reset() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
  
  void setto(float nx, float ny, float nz){
    xs = xt = x = nx;
    ys = yt = y = ny;
    zs = zt = z = nz;
  }

  void setto(vector v){
    xs = xt = x = v.x;
    ys = yt = y = v.y;
    zs = zt = z = v.z;
  }
  
  boolean equalegS(vector v){
    boolean isEqual = true;
    if(x != v.x || y != v.y || z != v.z) isEqual = false;
    return isEqual;
  }
  
  float move2D(float pps) {
    float totalDistance = sqrt(sq(xt-xs)+sq(yt-ys));
    //if (oldmillis != millis()/10%2) {
      pps = 30/pps; 
      float dx = xt-x;
      float dy = yt-y;
      float distance = sqrt(dx*dx+dy*dy);
      float angle = abs(atan(dy/dx));
      if (distance > totalDistance/pps) { //move step towards tpos
        if(dx>0) x += cos(angle)*totalDistance/pps;
        else x -= cos(angle)*totalDistance/pps;
        if(dy>0) y += sin(angle)*totalDistance/pps;
        else y -= sin(angle)*totalDistance/pps;
      }
      else {
        x = xt;
        y = yt;
      }
    //}
    distance = sqrt(sq(xt-x)+sq(yt-y));
    
    float distanceLeft = distance/totalDistance;
    if(totalDistance == 0 ) distanceLeft = 0;

    //oldmillis = (millis()/10%2); //static
    return distanceLeft;
  }
}
