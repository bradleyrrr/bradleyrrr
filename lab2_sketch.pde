/**
 **********************************************************************************************************************
 * @file       lab2_sketch.pde
 * @author     Bradley Rey (adapted from: Elie Hymowitz, Steve Ding, Colin Gallacher)
 * @version    V1.0
 * @date       05-Feb-2021
 * @brief      Lab 2 maze game example using 2-D physics engine from Haply and Fisica
 ***********************************************************************************************************************/

/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/

/* device block definitions ********************************************************************************************/
Board haplyBoard;
Device widgetOne;
Mechanisms pantograph;
byte widgetOneID = 5;
int CW = 0;
int CCW = 1;
boolean renderingForce = false;
/* end device block definition *****************************************************************************************/

/* framerate definition ************************************************************************************************/
long baseFrameRate = 120;
/* end framerate definition ********************************************************************************************/

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float pixelsPerCentimeter = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector angles = new PVector(0, 0);
PVector torques = new PVector(0, 0);

/* task space */
PVector posEE = new PVector(0, 0);
PVector fEE = new PVector(0, 0); 

/* World boundaries */
// --------------------------------------------------------------------------------------------- CHANGED WORLD BOUNDARIES TO ALLOW FOR SLIGHTLY BIGGER MAZE SPACE ---------------------------------------------------------------------------------------------
FWorld world;
float worldWidth = 35.0; 
float worldHeight = 20.0; 

float edgeTopLeftX = 0.0; 
float edgeTopLeftY = 0.0; 
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight;

float gravityAcceleration = 980; //cm/s2 was 980


/* Initialization of virtual tool */
HVirtualCoupling s;

/* define maze blocks */
FBox b1; // border 1
FBox b2; // border 2
FBox b3; // border 3
FBox b4; // border 4
FBox b5; // border 5
FBox l1; // water layer
FBox s1; // space layer
FBox s2; // sand layer
FBox r1; // red reset maze box 1
FBox r2; // red reset maze box 2
FBox r3; // red reset maze box 3
FBox r4; // red reset maze box 4
FBox r5; // red reset maze box 5
FBox sc1; // sand castle piece 1
FBox sc2; // sand castle piece 2
FBox sc3; // sand castle piece 3
FBox sc4; // sand castle piece 4
FBox sc5; // sand castle piece 5
FBox sc6; // sand castle piece 6
FBox sb1; // sail boat piece 1
FBox sb2; // sail boat piece 2
FBox sb3; // sail boat piece 3
FBox sb4; // sail boat piece 4
FBox sb5; // sail boat piece 5
FBox sat1; // satellite piece 1
FBox sat2; // satellite piece 2
FBox sat3; // satellite piece 3
FBox sat4; // satellite piece 4
FBox rk1; // rock 1
FBox rk2; // rock 2
FBox rk3; // rock 3
FBox fs1; // flagstick 1

boolean boxHasTouchedSand = false;
boolean ballHasTouchedSand = false;


/* define start and stop button */
FCircle c1; // start button
FBox c2; // stop button

/* define game ball */
FCircle g2; // game ball
FBox g1; // game box
FBox g3; // game box

/* define game start */
boolean gameStart = false;
boolean gameWin = false;
/* text font */
PFont f;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  // --------------------------------------------------------------------------------------------- CHANGED SETUP SIZE AS WIDTH AND HEIGHT MULTIPLIED BY PIXELS/CM ---------------------------------------------------------------------------------------------
  size(1400, 800); // was (1000, 400)

  /* set font type and size */
  f = createFont("Arial", 16, true);

  /* device setup */
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  // --------------------------------------------------------------------------------------------- CHANGE PORT HERE FOR YOUR SYSTEM --------------------------------------------------------------------------------------------- 
  haplyBoard  = new Board(this, "/dev/cu.usbmodem143201", 0);
  widgetOne = new Device(widgetOneID, haplyBoard);
  pantograph = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);

  widgetOne.device_set_parameters();

  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world = new FWorld();

  // ----------------------------------------------------------------------------------------------------------------- MAZE COMPONENTS -----------------------------------------------------------------------------------------------------------------

  setupWater();
  setupBoat();
  setupSand();
  setupSpace();
  setupGameRestart();
  setupPlayingObjects();

  /* Setup the Virtual Coupling Contact Rendering Technique */
  s = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(3); 
  s.h_avatar.setFill(255, 0, 0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);


  //drawStars();
  world.draw();

  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    textFont(f, 22);

    if (gameStart) {
      fill(0, 85, 255);
      textAlign(CENTER);
      text("Push the ball or square to the green flag to win!", width/2, 250);
      textAlign(CENTER);
      text("Touch the sun (yellow circle) to reset.", width/2, 280);
      text("Game Over if you touch a red object!", width/2, 310);
    } else {
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch the sun (yellow circle) to start the maze", width/2, 60);
    }
    
    if (gameWin) {
      fill(245, 66, 245);
      textAlign(CENTER);
      text("You won, congrats!! Touch the sun (yellow circle) to try again.", width/2, 250);
    }

    world.draw();
  }
}
/* end draw section ****************************************************************************************************/

void setupBorders () {
  //b1 = new FBox(1.0, 5.0);
  //b1.setPosition(edgeTopLeftX+worldWidth/4.0-2, edgeTopLeftY+worldHeight/2+1.5); 
  //b1.setFill(0);
  //b1.setNoStroke();
  //b1.setStaticBody(true);
  //world.add(b1);

  //b2 = new FBox(1.0, 5.0);
  //b2.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2-1.5); 
  //b2.setFill(0);
  //b2.setNoStroke();
  //b2.setStaticBody(true);
  //world.add(b2);

  //b3 = new FBox(1.0, 3.0);
  //b3.setPosition(edgeTopLeftX+worldWidth/4.0+8, edgeTopLeftY+worldHeight/2+1.5); 
  //b3.setFill(0);
  //b3.setNoStroke();
  //b3.setStaticBody(true);
  //world.add(b3);

  //b4 = new FBox(1.0, 5.0);
  //b4.setPosition(edgeTopLeftX+worldWidth/4.0+12, edgeTopLeftY+worldHeight/2-1.5); 
  //b4.setFill(0);
  //b4.setNoStroke();
  //b4.setStaticBody(true);
  //world.add(b4);

  //b5 = new FBox(3.0, 2.0);
  //b5.setPosition(edgeTopLeftX+worldWidth/2.0, edgeTopLeftY+worldHeight/2.0+2);
  //b5.setFill(0);
  //b5.setNoStroke();
  //b5.setStaticBody(true);
  //world.add(b5);
}

//void drawStars() {
//  float maxX = worldWidth;
//  float maxY= 6;
//  float maxDiameter = 1;
//  fill(255,255,255);
//  for (int i=0; i<100; i++) {
//    float size = random(maxDiameter);
//    ellipse(random(maxX), random(maxY), size, size);
//  }
//}

void setupWater () {
  /* Set water layer */
  l1 = new FBox(worldWidth/2+0.5, 5);
  l1.setPosition(worldWidth/4+1, worldHeight-3.25);
  l1.setFill(150, 150, 255, 80);
  l1.setDensity(100);
  l1.setSensor(true);
  l1.setNoStroke();
  l1.setStatic(true);
  l1.setName("Water");
  world.add(l1);

  rk1 = new FBox(3.0, 2.0);
  rk1.setPosition(edgeTopLeftX+6, worldHeight-1);
  rk1.setFill(0, 25, 25, 90);
  rk1.setNoStroke();
  rk1.setStaticBody(true);
  world.add(rk1);

  rk2 = new FBox(4.0, 4.0);
  rk2.setPosition(edgeTopLeftX+12, worldHeight-1.5);
  rk2.setFill(0, 25, 25, 90);
  rk2.setNoStroke();
  rk2.setStaticBody(true);
  world.add(rk2);

  rk3 = new FBox(0.5, 4.0);
  rk3.setPosition(edgeTopLeftX+14.25, worldHeight-2.5);
  rk3.setFill(0, 25, 25, 90);
  rk3.setNoStroke();
  rk3.setStaticBody(true);
  world.add(rk3);
}

void setupBoat () {
  sb1 = new FBox(6.0, 2.0);
  sb1.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2.0+3.25);
  sb1.setFill(175, 175, 175);
  sb1.setNoStroke();
  sb1.setStaticBody(true);
  world.add(sb1);

  sb2 = new FBox(1, 7);
  sb2.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2.0-0.5);
  sb2.setFill(175, 175, 175);
  sb2.setNoStroke();
  sb2.setStaticBody(true);
  world.add(sb2);

  sb3 = new FBox(1.5, 1);
  sb3.setPosition(edgeTopLeftX+worldWidth/4.0+1, edgeTopLeftY+worldHeight/2.0-2);
  sb3.setFill(175, 175, 175);
  sb3.setNoStroke();
  sb3.setStaticBody(true);
  world.add(sb3);

  sb4 = new FBox(2.5, 1);
  sb4.setPosition(edgeTopLeftX+worldWidth/4.0+1, edgeTopLeftY+worldHeight/2.0-1);
  sb4.setFill(175, 175, 175);
  sb4.setNoStroke();
  sb4.setStaticBody(true);
  world.add(sb4);

  sb5 = new FBox(3, 1);
  sb5.setPosition(edgeTopLeftX+worldWidth/4.0+1.25, edgeTopLeftY+worldHeight/2.0);
  sb5.setFill(175, 175, 175);
  sb5.setNoStroke();
  sb5.setStaticBody(true);
  world.add(sb5);
}

void setupSand () {
  /* Set sand layer */
  s2 = new FBox(worldWidth/2+0.5, 5);
  s2.setPosition(worldWidth/4+worldWidth/2-1, worldHeight-3.25);
  s2.setFill(255, 238, 153);
  s2.setDensity(4);
  s2.setNoStroke();
  s2.setStatic(true);
  s2.setName("Sand");
  world.add(s2);

  sc1 = new FBox(4, 1);
  sc1.setPosition(worldWidth-worldWidth/3-2, worldHeight-worldHeight/3.0+0.5);
  sc1.setFill(255, 238, 153);
  sc1.setNoStroke();
  sc1.setStaticBody(true);
  world.add(sc1);

  sc2 = new FBox(3, 1);
  sc2.setPosition(worldWidth-worldWidth/3+0.5-2, worldHeight-worldHeight/3.0-0.5);
  sc2.setFill(255, 238, 153);
  sc2.setNoStroke();
  sc2.setStaticBody(true);
  world.add(sc2);

  sc3 = new FBox(2, 3);
  sc3.setPosition(worldWidth-worldWidth/3+1-2, worldHeight-worldHeight/3.0-1.5);
  sc3.setFill(255, 238, 153);
  sc3.setNoStroke();
  sc3.setStaticBody(true);
  world.add(sc3);

  sc4 = new FBox(4, 1);
  sc4.setPosition(worldWidth-worldWidth/3+6, worldHeight-worldHeight/3.0+0.5);
  sc4.setFill(255, 238, 153);
  sc4.setNoStroke();
  sc4.setStaticBody(true);
  world.add(sc4);

  sc5 = new FBox(3, 4);
  sc5.setPosition(worldWidth-worldWidth/3+6.5, worldHeight-worldHeight/3.0-0.5);
  sc5.setFill(255, 238, 153);
  sc5.setNoStroke();
  sc5.setStaticBody(true);
  world.add(sc5);

  fs1 = new FBox(0.1, 2);
  fs1.setPosition(worldWidth-worldWidth/3+7.5, worldHeight-worldHeight/3.0-3.5);
  fs1.setFill(0);
  fs1.setNoStroke();
  fs1.setStaticBody(true);
  world.add(fs1);
}

void setupSpace () {
  /* Set space layer */
  s1 = new FBox(worldWidth, 5);
  s1.setPosition(worldWidth/2, edgeTopLeftY+2.5);
  s1.setFill(0, 0, 0, 99.9);
  s1.setDensity(0);
  s1.setSensor(true);
  s1.setNoStroke();
  s1.setStatic(true);
  s1.setName("Space");
  world.add(s1);

  sat1 = new FBox(0.1, 4);
  sat1.setPosition(edgeTopLeftX+worldWidth/2-3, edgeTopLeftY+3);
  sat1.setRotation(2.05);
  sat1.setFill(0);
  sat1.setNoStroke();
  sat1.setStaticBody(true);
  world.add(sat1);

  sat2 = new FBox(0.1, 7);
  sat2.setPosition(edgeTopLeftX+worldWidth/2+6, edgeTopLeftY+3.5);
  sat2.setRotation(1.76);
  sat2.setFill(0);
  sat2.setNoStroke();
  sat2.setStaticBody(true);
  world.add(sat2);
}

void setupGameRestart () {
  r1 = new FBox(2.5, 0.25); // diameter is 2
  r1.setPosition(edgeTopLeftX+8.75, worldHeight-1.85);
  r1.setFill(255, 0, 0);
  r1.setStaticBody(true);
  world.add(r1);

  r2 = new FBox(1.9, 0.25); // diameter is 2
  r2.setPosition(worldWidth-worldWidth/3+3, worldHeight-worldHeight/3.0+0.75);
  r2.setFill(255, 0, 0);
  r2.setStaticBody(true);
  world.add(r2);

  r3 = new FBox(1, 1); // diameter is 2
  r3.setPosition(edgeTopLeftX+worldWidth/2-3, edgeTopLeftY+3);
  r3.setRotation(0.5);
  r3.setFill(255, 0, 0);
  r3.setStaticBody(true);
  world.add(r3);

  r4 = new FBox(1, 1); // diameter is 2
  r4.setPosition(edgeTopLeftX+worldWidth/2+6, edgeTopLeftY+3.5);
  r4.setRotation(0.2);
  r4.setFill(255, 0, 0);
  r4.setStaticBody(true);
  world.add(r4);

  r5 = new FBox(0.25, 2); // diameter is 2
  r5.setPosition(worldWidth/2-1.4, worldHeight/2+6);
  r5.setFill(255, 0, 0);
  r5.setStaticBody(true);
  world.add(r5);
}

void setupPlayingObjects () {
  /* Start Button */
  c1 = new FCircle(3); // diameter is 2
  c1.setPosition(edgeTopLeftX+3, edgeTopLeftY+2.5);
  c1.setFill(255, 255, 50);
  c1.setStaticBody(true);
  world.add(c1);

  /* Finish Button */
  c2 = new FBox(1, 0.5);
  c2.setPosition(worldWidth-4.675, edgeTopLeftY+worldHeight/2.0-1);
  c2.setFill(25, 255, 25);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);

  /* Game Box */
  g1 = new FBox(0.5, 5);
  g1.setPosition(3, 10);
  g1.setDensity(10);
  g1.setFill(0, 85, 255);
  g1.setName("Widget");
  world.add(g1);

  /* Game Ball */
  g2 = new FCircle(2);
  g2.setPosition(2, 4);
  g2.setDensity(10);
  g2.setFill(0, 85, 255);
  g2.setName("Widget");
  world.add(g2);
  
  /* Game Box */
  g3 = new FBox(0.25, 8);
  g3.setPosition(24.3, 10);
  g3.setRotation(1.57);
  g3.setDensity(150);
  g3.setDamping(300);
  g3.setFill(0, 85, 255);
  g3.setName("Widget");
  world.add(g3);
}


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));
    }
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();


    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    if (s.h_avatar.isTouchingBody(c1)) {
      gameStart = true;
      gameWin = false;
      g1.setPosition(2, 14);
      g2.setPosition(3, 8);
      g3.setPosition(24.3, 10);
      s.h_avatar.setSensor(false);
      boxHasTouchedSand = false;
      ballHasTouchedSand = false;
      g1.setDensity(10);
      g2.setDensity(7);
    }

    // if win
    if (g1.isTouchingBody(c2) || g2.isTouchingBody(c2)) {
      gameWin = true;
      gameStart = false;
      s.h_avatar.setSensor(true);
    }

    // if lose
    if (g1.isTouchingBody(r1) || g2.isTouchingBody(r1) || g3.isTouchingBody(r1) || g1.isTouchingBody(r2) || g2.isTouchingBody(r2) || g3.isTouchingBody(r2) || g1.isTouchingBody(r3) || g2.isTouchingBody(r3) || g3.isTouchingBody(r4)
    || g1.isTouchingBody(r4) || g2.isTouchingBody(r4) || g3.isTouchingBody(r4) || g1.isTouchingBody(r5) || g2.isTouchingBody(r5) || g3.isTouchingBody(r5)) {
      gameStart = true;
      g1.setPosition(2, 14);
      g2.setPosition(3, 8);
      g3.setPosition(24.3, 10);
      s.h_avatar.setSensor(false);
      g1.resetForces();
      g2.resetForces();
      g1.setDensity(10);
      g2.setDensity(10);
      g1.setDamping(20);
      g2.setDamping(20);
    }

    /* water layer codes */
    if (s.h_avatar.isTouchingBody(l1)) {
        s.h_avatar.setDamping(825);
    } else if (s.h_avatar.isTouchingBody(s1)) {
        s.h_avatar.setDamping(10);
    } else if (s.h_avatar.isTouchingBody(s2)) {
        s.h_avatar.setDamping(500);
    }

    // if both are touching the water
    if (gameStart && g1.isTouchingBody(l1) && g2.isTouchingBody(l1)) {
      g1.setDamping(20);
      g2.setDamping(20);
    }
    
    // if ball is touching space
    if (g2.isTouchingBody(s1)) {
      g2.setDensity(1);
      g2.setDamping(1);
      g2.setFriction(3);
      g2.addForce(0, (l1.getDensity()*sq(g2.getSize())*gravityAcceleration*-1)/127.26);
      g2.setRestitution(-100);
    } 
    
    // if ball is touching sand
    if (g2.isTouchingBody(s2)) {
      g2.setDamping(50);
      g2.setDensity(15);
      ballHasTouchedSand = true;
    }
    
    // if ball is on boat
    if (g2.isTouchingBody(sb2)) {
       g2.setDamping(10); 
    }

    // if box is touching ball
    if (g1.isTouchingBody(g2) && boxHasTouchedSand) {
      g1.setRestitution(0);
    }

    // if box is now touching the sandcastle
    if (g1.isTouchingBody(sc1)) {
      g1.setDensity(10);
      boxHasTouchedSand = true;
    }
    
    if (g3.isTouchingBody(g2)) {
      g2.setDamping(10);
      g3.setDamping(10);
      g2.setDensity(1);
      g3.setDensity(1);
    }
    
    if (ballHasTouchedSand) {
      g2.setDamping(50);
      g2.setDensity(15);
    }


    /* Bouyancy of fluid on avatar and gameball section */
    if (g1.isTouchingBody(l1)) {
      float b_s;
      float bm_d = g1.getY()-l1.getY()+l1.getHeight()/2; // vertical distance between middle of ball and top of water

      if (bm_d + g1.getWidth()/2 >= g1.getWidth()) { //if whole ball or more is submerged
        b_s = g1.getWidth(); // amount of ball submerged is ball size
      } else { //if ball is partially submerged
        b_s = bm_d + g1.getWidth()/2; // amount of ball submerged is vertical distance between middle of ball and top of water + half of ball size
      }

      //g1.addForce(0,l1.getDensity()*sq(b_s)*gravityAcceleration*-1); // 300 is gravity force
      g1.addForce(0, (l1.getDensity()*sq(b_s)*gravityAcceleration*-1)/1);
    }

    if (g2.isTouchingBody(l1)) {
      float b_s;
      float bm_d = g2.getY()-l1.getY()+l1.getHeight()/2; // vertical distance between middle of ball and top of water

      if (bm_d + g2.getSize()/2 >= g2.getSize()) { //if whole ball or more is submerged
        b_s = g2.getSize(); // amount of ball submerged is ball size
      } else { //if ball is partially submerged
        b_s = bm_d + g2.getSize()/2; // amount of ball submerged is vertical distance between middle of ball and top of water + half of ball size
      }

      //g2.addForce(0,l1.getDensity()*sq(b_s)*gravityAcceleration*-1); // 300 is gravity force
      g2.addForce(0, (l1.getDensity()*sq(b_s)*gravityAcceleration*-1)/4);
    }
    /* End Bouyancy of fluid on avatar and gameball section */


    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* Alternate bouyancy of fluid on avatar and gameball helper functions, comment out
 * "Bouyancy of fluid on avatar and gameball section" in simulation and uncomment 
 * the helper functions below to test
 */

/*
void contactPersisted(FContact contact){
 float size;
 float b_s;
 float bm_d;
 
 if(contact.contains("Water", "Widget")){
 size = 2*sqrt(contact.getBody2().getMass()/contact.getBody2().getDensity()/3.1415);
 bm_d = contact.getBody2().getY()-contact.getBody1().getY()+l1.getHeight()/2;
 
 if(bm_d + size/2 >= size){
 b_s = size;
 }
 else{
 b_s = bm_d + size/2;
 }
 
 contact.getBody2().addForce(0, contact.getBody1().getDensity()*sq(b_s)*300*-1);
 contact.getBody2().setDamping(20);
 }
 
 }
 
 
 void contactEnded(FContact contact){
 if(contact.contains("Water", "Widget")){
 contact.getBody2().setDamping(0);
 }
 }
 */

/* End Alternate Bouyancy of fluid on avatar and gameball helper functions */

/* end helper functions section ****************************************************************************************/
