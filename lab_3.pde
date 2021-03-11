/**
 **********************************************************************************************************************
 * @file       lab3_sketch.pde
 * @author     Bradley Rey (adapted from: Elie Hymowitz, Steve Ding, Colin Gallacher)
 * @version    V1.0
 * @date       12-March-2021
 * @brief      Lab 3 communicating with Haply via force feedback and PID control
 ***********************************************************************************************************************/

/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import controlP5.*;
/* end library imports *************************************************************************************************/

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/

ControlP5 cp5;

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
float pixelsPerMeter = 4000.0;
float radsPerDegree = 0.01745;

/* pantagraph link parameters in meters */
float l = 0.07;
float L = 0.09;


/* end effector radius in meters */
float rEE = 0.006;

/* generic data for a 2DOF device */
/* joint space */
PVector angles = new PVector(0, 0);
PVector torques = new PVector(0, 0);
PVector oldangles = new PVector(0, 0);
PVector diff = new PVector(0, 0);

/* task space */
PVector posEE = new PVector(0, 0);
PVector fEE = new PVector(0, 0); 

/* device graphical position */
PVector deviceOrigin = new PVector(0, 0);

float x_m,y_m;

// used to compute the time difference between two loops for differentiation
long oldtime = 0;
// for changing update rate
int iter = 0;
int pathIter = 0;
int degrees = 0;

int loopTimeIter = 0;

/// PID stuff

float P = 0.0;
// for I
float I = 0;
float cumerrorx = 0;
float cumerrory = 0;
// for D
float oldex = 0.0f;
float oldey = 0.0f;
float D = 0;

//for exponential filter on differentiation
float diffx = 0;
float diffy = 0;
float buffx = 0;
float buffy = 0;
float smoothing = 0.80;

float xr = 0;
float yr = 0;

// checking everything run in less than 1ms
long timetaken= 0;

// set loop time in usec (note from Antoine, 500 is about the limit of my computer max CPU usage)
int looptime = 500;

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

/* setup world object */
FBox safeWall; // safe wall

FBox scaredBox; // scared box

FBox damp1; // dampened layer 1
FBox damp2; // dampened layer 2
FBox damp3; // dampened layer 3
FBox damp4; // dampened layer 4

boolean gameStart = false;

/* text font */
PFont f;

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

  //target = createShape(ELLIPSE, 0,0, 20, 20);
  //target.setStroke(color(0));
  
  //xr = random(-0.5,0.5);
  //yr = random(-0.5,0.5);

  // ----------------------------------------------------------------------------------------------------------------- MAZE COMPONENTS -----------------------------------------------------------------------------------------------------------------

  setupSafeZone();
  setupNervousZone();
  setupScaredBox();
  setupAgitatedZone();

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


void keyPressed() {
  if (key == 'i') {
    widgetOne.device_set_parameters();
  } else if (key == ' ') {
    cumerrorx= 0;
    cumerrory= 0;
  }
}

/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    textFont(f, 22);
    //update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);

    world.draw();
  }
}
/* end draw section ****************************************************************************************************/

void setupSafeZone() {
  safeWall = new FBox(worldWidth/32, worldHeight/1.5);
  safeWall.setPosition(worldWidth/8, worldHeight/2);
  safeWall.setFill(0);
  safeWall.setDensity(100);
  safeWall.setNoStroke();
  safeWall.setStaticBody(true);
  safeWall.setName("SafeWall");
  world.add(safeWall);
}

void setupAgitatedZone() {
  for (int i=0; i<70; i++) {
    FBox temp = new FBox (random(0.5, worldWidth/32), random(0.5, worldHeight/32));
    temp.setPosition(random(worldWidth/8,worldWidth/2.5), random(0,worldHeight));
    temp.setFill(0);
    temp.setStatic(true);
    world.add(temp);
  }
}

void setupNervousZone() {
  damp1 = new FBox(worldWidth/8, worldHeight);
  damp1.setPosition(worldWidth/2, worldHeight/2);
  damp1.setFill(190, 220, 255, 90);
  damp1.setDensity(500);
  damp1.setSensor(true);
  damp1.setNoStroke();
  damp1.setStatic(true);
  damp1.setName("Damp1");
  world.add(damp1);
  
  damp2 = new FBox(worldWidth/8, worldHeight);
  damp2.setPosition(worldWidth/2+worldWidth/8, worldHeight/2);
  damp2.setFill(135, 200, 255, 90);
  damp2.setDensity(700);
  damp2.setSensor(true);
  damp2.setNoStroke();
  damp2.setStatic(true);
  damp2.setName("Damp2");
  world.add(damp2);
  
  damp3 = new FBox(worldWidth/8, worldHeight);
  damp3.setPosition(worldWidth/2+(2*worldWidth/8), worldHeight/2);
  damp3.setFill(85, 150, 255, 90);
  damp3.setDensity(850);
  damp3.setSensor(true);
  damp3.setNoStroke();
  damp3.setStatic(true);
  damp3.setName("Damp3");
  world.add(damp3);
  
  damp4 = new FBox(worldWidth/6, worldHeight);
  damp4.setPosition(worldWidth/2+(3*worldWidth/8)+0.75, worldHeight/2);
  damp4.setFill(0,115,250, 100);
  damp4.setDensity(900);
  damp4.setSensor(true);
  damp4.setNoStroke();
  damp4.setStatic(true);
  damp4.setName("Damp4");
  world.add(damp4);
}

void setupScaredBox() {
  scaredBox = new FBox(worldWidth/32, worldHeight/1.5);
  scaredBox.setPosition(worldWidth-worldWidth/12, worldHeight/2);
  scaredBox.setFill(0);
  scaredBox.setDensity(100);
  scaredBox.setNoStroke();
  scaredBox.setStatic(true);
  scaredBox.setName("ScaredBox");
  world.add(scaredBox);
}


int noforce = 0;
long timetook = 0;
long looptiming = 0;

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    s.h_avatar.setSensor(false);

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
    
    if (s.h_avatar.isTouchingBody(safeWall))
        gameStart = true;
    
    // if we touch the scared box we need to shoot back to the start
    if (s.h_avatar.isTouchingBody(scaredBox)) {
        gameStart = false;
        moveAvatar();
    }
    
    // damping of elements
    if (s.h_avatar.isTouchingBody(damp1))
        s.h_avatar.setDamping(900);
        
    else if (s.h_avatar.isTouchingBody(damp2))
        s.h_avatar.setDamping(950);
        
    else if (s.h_avatar.isTouchingBody(damp3))
        s.h_avatar.setDamping(975);
        
    else if (s.h_avatar.isTouchingBody(damp4))
        s.h_avatar.setDamping(990);
        
    else 
        s.h_avatar.setDamping(50);
    
    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

void moveAvatar() {
      println("test");
}

/* end helper functions section ****************************************************************************************/
