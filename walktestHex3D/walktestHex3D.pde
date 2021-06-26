/*
Hexapod Simulator
 2021 Maximilian Kern
 */

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

gaitEngine Hex = new gaitEngine();

byte[][] tripodGait = new byte[120][6];
byte[][] waveGait = new byte[120][6];

float camRotX; // Camera rotation and translation
float camRotY;
float camTraY;
float camTraX;

float jX = 0; // (Joystick) Walking X, Y and Rotate
float jY = 0;
float jR = 0;

float gX = 0; // Ground X, Y, and Rotation
float gY = 0;
float gR = 0;

boolean showUI = true;

PGraphics img;

PShape bodyObj, coxaObj, femurObj, tibiaObj, coxaMObj, femurMObj, tibiaMObj;

void setup() {

  size(700, 700, P3D); //, OPENGL)
  rectMode(CENTER);

  frameRate(50);

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


  unzip(sketchPath("assets.zip"));

  bodyObj = loadShape("Body.obj");
  coxaObj = loadShape("Coxa.obj");
  coxaMObj = loadShape("CoxaM.obj");
  femurObj = loadShape("Femur.obj");
  tibiaObj = loadShape("Tibia.obj");
  tibiaMObj = loadShape("TibiaM.obj");
  tibiaObj.scale(1000, 1000, 1000);
  tibiaMObj.scale(1000, 1000, 1000);
  coxaObj.scale(1000, 1000, 1000);
  coxaMObj.scale(1000, 1000, 1000);
  bodyObj.scale(1000, 1000, 1000);
  femurObj.scale(1000, 1000, 1000);

  img = createGraphics(500, 500);
  img.beginDraw();
  for (int i = 0; i < width; i ++) {
    for (int j = 0; j < height; j ++) {
      img.set(i, j, color(random(100, 200)));
    }
  }
  img.endDraw();

  stroke(255);
 
}

void draw() {
  translate(+width/2, +height/2);
  scale(2);
  translate(-width/2, -height/2);
  smooth(8);
  fill(0);
  background(255);
  lights();
  directionalLight(51, 102, 126, -1, 0, 0);
  directionalLight(51, 102, 126, 1, 0, 0);
  // Camera Rotation
  translate(width/2, height/2);

  rotateX(-camRotY);
  rotateZ(camRotX);
  
  jY = 0.2;
  
  // Calculate rotation and translation of ground below robot
  float sinG = sin(gR/100);
  float cosG = cos(gR/100); 
  gX -= jX*cosG - jY*sinG;
  gY += jY*cosG + jX*sinG;
  gR -= jR;


  if(showUI && camRotY > -PI/2){
    pushMatrix();
    rotate(gR/100);
    translate(gX-10*img.width/2, gY-10*img.height/2, -0.01);
    scale(10);
    image(img, 0, 0);
    popMatrix();
    fill(255);
    //text("Left Mouse Drag or [W][A][S][D] to move", 10, 20);
    //text("Right Mouse Drag or [Q][E] to rotate", 10, 40);
    //text("Middle Mouse Drag to look around", 10, 60); 
    //text("jX " + nf(jX, 0, 2) + " jY " + nf(jY, 0, 2) + " jR " + nf(jR, 0, 2), 10, 80);  
    for (int i = 0; i < 6; i++) {
      pushMatrix(); // Verify IK by using target coordinates directly
      fill(255);
      text(i, Hex.leg[i].x+10, Hex.leg[i].y+10);
      noFill();
      rect(Hex.legT[i].x, Hex.legT[i].y, 20, 20);
      rect(Hex.legS[i].x, Hex.legS[i].y, 20+Hex.targetDeadZone*2, 20+Hex.targetDeadZone*2);
      translate(Hex.leg[i].x, Hex.leg[i].y, Hex.leg[i].z);
      sphere(3);
      popMatrix();
    } 
  }
    
  //compensate for body translation/rotation
  rotateX(Hex.rotY);
  rotateY(Hex.rotX);
  rotateZ(Hex.rotZ);
  translate(-Hex.traX, -Hex.traY, -Hex.traZ);
  
  // Configure and run the hexapod engine
  Hex.gSeq = waveGait;
  Hex.walkX = jX;
  Hex.walkY = jY;
  Hex.walkR = jR;
  Hex.gaitStep();
  Hex.runBodyIK();
  Hex.runLegIK();
  
  
  if(showUI){
    // Draw body polygon
    beginShape(); 
    noFill();
    for (int i = 0; i < 6; i++) {
      vertex(Hex.coxaPos[i].x, Hex.coxaPos[i].y, Hex.coxaPos[i].z);
    } 
    endShape(CLOSE);
  
    // Draw leg support polygon
    beginShape(); 
    if(camRotY > -PI/2) fill(#62BBE3, 100);
    for (int i = 0; i < 6; i++) {
      if (Hex.legIK[i].z == 0) vertex(Hex.legIK[i].x, Hex.legIK[i].y);
    }    
    endShape(CLOSE);
  }

  pushMatrix();
  translate(0, 0, Hex.coxaPos[0].z + 26);
  shape(bodyObj);
  popMatrix();

  // Draw Legs
  for (int i = 0; i < 6; i++) {
    pushMatrix();
    translate(Hex.coxaPos[i].x, Hex.coxaPos[i].y, Hex.coxaPos[i].z);
    rotateZ(Hex.legAngle[i][0]);
    if (i>2) rotateZ(PI);
    pushMatrix();
    rotateZ(-PI/2);
    translate(0, 0, -3);
    if (i<3) {
      shape(coxaObj);
    } else {
      rotateY(PI);
      shape(coxaMObj);
    }
    popMatrix();
    if(showUI) line(0, 0, 0, -Hex.coxa, 0, 0);

    translate(-Hex.coxa, 0, 0);
    rotateY(PI-Hex.legAngle[i][1]);
    pushMatrix();
    rotateZ(-PI/2);
    rotateX(PI);
    if (i<3) {
      translate(14, 0, 0);
      shape(femurObj);
    } else {
      rotateY(PI);
      rotateX(PI);
      translate(14, 44, 0);
      shape(femurObj);
    }      
    popMatrix();
    if(showUI) line(0, 0, 0, Hex.femur, 0, 0);

    translate(Hex.femur, 0, 0);
    rotateY(-PI/2-Hex.legAngle[i][2]);
    pushMatrix();      
    if (i<3) {
      translate(0, 15, 0);
      rotateY(PI/2);
      rotateZ(PI);
      shape(tibiaMObj);
    } else {
      translate(0, -15, 0);
      rotateY(-PI/2);
      shape(tibiaObj);
    }
    popMatrix();
    if(showUI) line(0, 0, 0, Hex.tibia, 0, 0);      
    popMatrix();
    
  }
}

void mouseReleased() {
  jX = 0;
  jY = 0;
  jR = 0;
}

void mouseDragged()
{
  if (mouseButton == LEFT) {
    jX += (mouseX - pmouseX) * 0.01; 
    jY -= (mouseY - pmouseY) * 0.01;
  }  
  if (mouseButton == RIGHT) {
    jR += (mouseX - pmouseX) * 0.01;
  }
  if (mouseButton == CENTER) {
    camRotX -= (mouseX - pmouseX) * 0.01;
    camRotY += (mouseY - pmouseY) * 0.01;
  }
}

void keyPressed() {
  if (key == 'a') jX = -0.5; 
  if (key == 'd') jX = 0.5;
  if (key == 's') jY = -0.5;
  if (key == 'w') jY = 0.5;
  if (key == 'q') jR = -0.5;
  if (key == 'e') jR = 0.5;
  if (key == 'u') showUI = !showUI;
}

void keyReleased() {
  if (key == 'd' || key == 'a') jX = 0; 
  if (key == 's' || key == 'w') jY = 0;
  if (key == 'e' || key == 'q') jR = 0;
}

//source: https://www.journaldev.com/960/java-unzip-file-example
void unzip(String zipFilePath) {
  FileInputStream fis;
  byte[] buffer = new byte[1024];
  try {
    fis = new FileInputStream(zipFilePath);
    ZipInputStream zis = new ZipInputStream(fis);
    ZipEntry ze = zis.getNextEntry();
    while (ze != null) {
      String fileName = ze.getName();
      File newFile = new File(sketchPath("") + File.separator + fileName);
      FileOutputStream fos = new FileOutputStream(newFile);
      int len;
      while ((len = zis.read(buffer)) > 0) {
        fos.write(buffer, 0, len);
      }
      fos.close();
      zis.closeEntry();
      ze = zis.getNextEntry();
    }
    zis.closeEntry();
    zis.close();
    fis.close();
  } 
  catch (IOException e) {
    e.printStackTrace();
  }
}
