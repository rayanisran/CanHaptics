
/**
 **********************************************************************************************************************
 * @file       lab3.pde
 * @author     Rayan Isran
 * @version    V1.0.0
 * @date       20-April-2021
 * @brief      Lab 3
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 20.0;  
float             worldHeight                         = 17.5; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */

// Mode 1 stuff
int rows = 15;
int cols = 18;
FBox[][] fboxes = new FBox[rows][cols];

// Mode 2 stuff
FBox                l4;

/* define game ball */
FCircle           g2;
FBox              g1;

//String to define the mode; should probably be an enum but whatever
String k = "off";

/* text font */
PFont             f;
ScheduledFuture<?> handle;
/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */ 
  
  /* screen size definition */
  size(800, 700);
  
  /* set font type and size */
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
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
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
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  //This defines the Haply avatar the user controls.
  s = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(0); 
  s.h_avatar.setDamping(0);
  s.h_avatar.setFill(0,0,0);   // Make it "invisible!
  s.h_avatar.setSensor(false);
  
  //This defines the box we use in Mode 2 that acts as a fluid
  l4 = new FBox(15, 16);
  l4.setPosition(worldWidth/2, worldHeight/1.9);
  l4.setFill(0, 0, 0, 0);
  l4.setSensor(true);
  l4.setNoStroke();
  l4.setStatic(true);
  l4.setName("Mode2");
  
  //These are the boxes that define the grid in Mode 1
  for (int i = 0; i < cols; i++) // C = 17
  {
    for (int j = 0; j < rows; j++) //R = 14
    {
          fboxes[j][i] = createWall(0.3, 0.25, 1.75 + 1 * i, 15.5 - j + 1);
          fboxes[j][i].setFill(0, 0, 0);
          fboxes[j][i].setName("Mode1");
    }
  }

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), 0);
  world.draw();
    
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  handle = scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/

void exit() {
  handle.cancel(true);
  scheduler.shutdown();
  widgetOne.set_device_torques(new float[]{0, 0});
  widgetOne.device_write_torques();
  super.exit();
}

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(0);
    world.draw();    
    
    textAlign(CENTER);
    text("Press keys 1, 2, or 3 to change mode.", width/2, 30);
    textAlign(CENTER);
    text("Before changing modes, return to callibration point to make sure you start in the same position.", width/2 + 40, 60);
    fill(255, 0, 0);
    text("IMPORTANT: Please GENTLY hold the end-effector before switching to mode 3.", width/2 + 40, 80);
    fill(255, 255, 255);
    textAlign(LEFT);
    text("MODE: " + k, width/15, 60);
  }
}

void keyPressed() {
  /*reset*/
 
  if (key == '1') {
    
      removeBodyByName("Mode2"); // remove Mode 2 objects
      s.h_avatar.setDamping(0); // remove any damping set from other modes
    
      for (int i = 0; i < cols; i++)
      {
        for (int j = 0; j < rows; j++)
          world.add(fboxes[j][i]);
      }
      k = "1";
  }
  if (key == '2') {
      removeBodyByName("Mode1");
      world.add(l4);
      k = "2";
    }
  if (key == '3') {
      removeBodyByName("Mode1");
      removeBodyByName("Mode2");
      k = "3";
  }
}

Boolean bodyExists(FBody body) {
  ArrayList<FBody> bodies = world.getBodies();
  for (FBody b : bodies) {
    try {
      if (b == body) {
        return true;
      }
    } 
    catch(NullPointerException e) {
      // do nothing
    }
  }
  return false;
}
void removeBodyByName(String bodyName) {
  ArrayList<FBody> bodies = world.getBodies();
  for (FBody b : bodies) {
    try {
      if (b.getName().equals(bodyName)) {
        world.remove(b);
      }
    } 
    catch(NullPointerException e) {
      // do nothing
    }
  }
}

//stuff for mode 1 simulation
int mode1_duration[] = { 10, 12, 25 };
float mode1_tLastChange = Float.NEGATIVE_INFINITY;
int mode1_index = 0;

//stuff for mode 2 simulation
int[] mode2_thresholds =    { 6, 7, 9, 11, 14, 16, 19 };
int[] mode2_durations = { 60, 50, 40, 30, 20, 16, 12};
float mode2_tLastChange = Float.NEGATIVE_INFINITY;
int mode2_index = 0;
int mode2_damping = 0;

PVector mode2_posEELast = new PVector(0, 0);
float mode2_threshold = 0.02;
float MODE2_graph_Y = 61.4;
float MODE2_graph_C = 36.4;

//stuff for mode 3 simulation
PVector[] mode3_locations = {
  new PVector(-0.055, 0.0383),
  new PVector(-0.022, 0.0724),
  new PVector(0.0184, 0.0810),
  new PVector(0.0588, 0.0724),
  new PVector(0.0810, 0.0383),
  new PVector(0.0588, 0.0724),
  new PVector(0.0184, 0.0810),
  new PVector(-0.022, 0.0724),
};


float mode3_tLastChange = Float.NEGATIVE_INFINITY;
int mode3_state = 0; // 0 for static, 1 for moving
float mode3_tDuration = 1000; // ms
float mode3_tVib = 40; // ms, how long to vibrate the Haply before moving to next point
int mode3_index;

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
   
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));

      // for mode 3 we don't want the Haply avatar to follow us around but instead use point-based navigation
      if (k == "3")
        posEE.set(device_to_graphics(posEE)); 
      else
      {
        posEE.set(posEE.copy().mult(200));  
        s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-4); 
        s.updateCouplingForce();   
          
        fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
        fEE.div(100000); //dynes to newtons 
      }
    }

    // properties based on mode
    // for mode 1, just apply some random texture as the user moves around
    if (k == "1")
    {
        mode1_index = int(random(0, mode1_duration.length - 1));
        if (millis() - mode1_tLastChange > mode1_duration[mode1_index])
        {    
          fEE.x = random(-1, 1);
          mode1_tLastChange = millis();
        }
    }
    else if (k == "2")
    {
      //println("x-pos: " + posEE.x + ", y-pos: " + posEE.y);
      
      if (s.h_avatar.isTouchingBody(l4)) 
      {
        PVector xDiff = (posEE.copy()).sub(mode2_posEELast);
        mode2_posEELast.set(posEE);
        
        // only if we're in contact with the body and have moved just enough
        if ((xDiff.mag()) < mode2_threshold) 
        { 
          // set a damping value based on our equation
          mode2_damping = round(MODE2_graph_Y * posEE.y + MODE2_graph_C);
          s.h_avatar.setDamping(mode2_damping); 
          
          // some vibrations
          for (int i = mode2_thresholds.length - 1; i >=0; i--)
          {
            if (posEE.y > mode2_thresholds[i] && mode2_index != mode2_thresholds[i])
              mode2_index = mode2_thresholds[i];
          }

          if (millis() - mode2_tLastChange > mode2_durations[mode2_index])
          {    
            fEE.x = random(-5, 5);
            fEE.y = random(-1, 1);
            mode2_tLastChange = millis();
          }
        }
      }     
    }
    
    //for k = 3
    //state 0 = vibration state
    //state 1 = navigation state
    else if (k == "3")
    {
      PVector force = new PVector(0, 0);

        // if it has been an entire second, then switch to the next index
        if (millis() - mode3_tLastChange > mode3_tDuration) 
        {
          if (mode3_index == mode3_locations.length - 1)
            mode3_index = -1;
          mode3_index++;
          
          mode3_tLastChange = millis();
          mode3_state = 1;
        }
        // just before navigating to the next point we'll vibrate the Haply briefly to simulate a tick sound/effect
        else if (millis() - mode3_tLastChange > mode3_tDuration - mode3_tVib)
        {
            mode3_state = 0;
            fEE.x = random(-10, 10);
        }
        
        if (mode3_state != 0)
        { 
          PVector xDiff = (posEE.copy()).sub(mode3_locations[mode3_index]);
          force.set(xDiff.mult(-400)); 
          fEE.set(graphics_to_device(force)); 
        }
;
  
    }
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}

/* end simulation section **********************************************************************************************/

FBox createWall(float w, float h, float x, float y){
  FBox wall = new FBox(w,h);
  wall.setPosition(x, y);
  wall.setNoStroke();
  wall.setStatic(true);
  wall.setFill(204, 0, 102);
  //wall.setRestitution(0.5);
  return wall;
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
