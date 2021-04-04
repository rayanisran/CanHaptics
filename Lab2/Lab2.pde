
/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Rayan Isran
 * @version    V1.0.0
 * @date       03-April-2021
 * @brief      Maze example using 2-D physics engine
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

float h, v;
int rows = 14;
int cols = 17;

FBox[][] fboxes_h = new FBox[rows][cols];
FBox[][] fboxes_v = new FBox[rows][cols];


/* define start and stop button */
FCircle           c1;
FCircle           c2;

FCircle blob1 = new FCircle(0.75);
FCircle blob2 = new FCircle(0.75);
FCircle blob3 = new FCircle(0.75);
FCircle blob4 = new FCircle(0.75);
FCircle blob5 = new FCircle(0.75);
FCircle blob6 = new FCircle(0.75);
FCircle blob7 = new FCircle(0.75);


/* define game ball */
FCircle           g2;
FBox              g1;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  

//ORIGINAL:
//int[][][] maze = new int[][][] {
//{{1,1}, {1,0}, {1,0}, {1,1}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {0, 1}},
//{{1,1}, {0,0}, {1,0}, {0,1}, {1,0}, {1,0}, {1,1}, {1,0}, {0,1}, {1,0}, {1,0}, {0,1}, {1,0}, {0,1}, {0,1}, {1,0}, {0, 1}}, 
//{{0,1}, {1,0}, {0,1}, {1,0}, {1,1}, {0,0}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {1,0}, {0,0}, {0,1}, {1,1}, {0,0}, {0, 1}}, 
//{{0,1}, {1,1}, {1,0}, {0,1}, {0,0}, {0,1}, {0,1}, {1,0}, {0,1}, {0,1}, {1,0}, {1,0}, {1,0}, {0,1}, {0,0}, {1,1}, {0, 1}}, 
//{{1,1}, {0,0}, {0,1}, {1,1}, {1,0}, {1,0}, {1,0}, {0,1}, {0,0}, {1,1}, {0,0}, {1,0}, {0,1}, {1,0}, {1,0}, {0,1}, {0, 1}}, 
//{{0,1}, {1,1}, {0,0}, {0,0}, {1,1}, {1,0}, {0,1}, {1,0}, {1,1}, {1,1}, {1,0}, {0,0}, {0,1}, {1,0}, {0,1}, {0,1}, {0, 1}}, 
//{{0,1}, {1,0}, {1,1}, {1,0}, {0,1}, {1,0}, {1,0}, {0,0}, {0,1}, {0,0}, {1,1}, {0,0}, {1,1}, {0,0}, {0,1}, {0,1}, {0, 1}}, 
//{{1,1}, {0,0}, {0,1}, {1,0}, {0,0}, {0,1}, {1,0}, {1,1}, {0,0}, {0,1}, {0,1}, {1,0}, {1,0}, {1,1}, {0,1}, {0,0}, {0, 1}}, 
//{{0,1}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {0,1}, {1,0}, {1,0}, {1,0}, {1,1}, {1,0}, {0,0}, {0,1}, {1,0}, {0,0}, {0, 1}}, 
//{{1,1}, {1,0}, {0,1}, {1,1}, {1,0}, {0,1}, {0,1}, {1,1}, {1,0}, {0,1}, {0,0}, {1,1}, {1,0}, {0,0}, {0,1}, {1,1}, {0, 1}}, 
//{{0,1}, {1,0}, {0,0}, {0,1}, {0,1}, {0,1}, {1,0}, {0,0}, {0,1}, {1,1}, {1,1}, {1,0}, {1,0}, {0,1}, {0,1}, {0,0}, {0, 1}}, 
//{{1,1}, {0,1}, {1,1}, {0,0}, {1,1}, {0,0}, {1,1}, {1,0}, {0,1}, {0,1}, {0,1}, {1,1}, {0,0}, {0,1}, {0,1}, {1,0}, {0, 1}}, 
//{{0,1}, {0,0}, {0,0}, {1,0}, {0,1}, {1,0}, {0,0}, {1,1}, {0,0}, {0,1}, {0,0}, {0,1}, {1,0}, {0,0}, {1,1}, {0,0}, {0, 1}},
//{{1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {0,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {0, 1}}
//};  
//
//2,7
int[][][] maze = new int[][][] 
{
{{0,0}, {1,1}, {0,1}, {0,1}, {1,1}, {0,1}, {0,1}, {1,1}, {0,1}, {1,1}, {0,1}, {1,1}, {0,1}, {1,0}}, // cols = 13, rows = 16
{{1,0}, {0,0}, {1,0}, {1,1}, {0,0}, {1,1}, {1,0}, {0,0}, {1,0}, {1,0}, {1,0}, {0,1}, {0,0}, {1,0}},
{{1,0}, {1,0}, {0,1}, {1,0}, {0,1}, {0,0}, {1,1}, {0,0}, {1,0}, {0,1}, {0,0}, {1,1}, {0,0}, {1,0}},
{{1,1}, {0,1}, {1,0}, {0,1}, {1,1}, {0,0}, {1,0}, {1,0}, {1,0}, {1,1}, {0,1}, {0,0}, {1,0}, {1,0}},
{{1,0}, {1,0}, {1,1}, {0,0}, {1,0}, {1,1}, {0,1}, {0,0}, {1,0}, {1,0}, {0,1}, {1,1}, {0,1}, {1,0}},
{{1,0}, {1,0}, {0,0}, {0,1}, {1,0}, {1,0}, {1,0}, {0,1}, {1,0}, {0,1}, {0,1}, {0,0}, {1,0}, {1,0}},
{{1,0}, {1,1}, {0,1}, {0,1}, {1,0}, {0,1}, {1,0}, {1,0}, {0,1}, {0,1}, {1,0}, {1,1}, {0,0}, {1,0}},
{{1,0}, {1,0}, {0,1}, {1,0}, {0,1}, {1,0}, {0,0}, {1,1}, {1,0}, {1,1}, {0,0}, {1,0}, {1,1}, {1,0}},
{{1,0}, {0,1}, {0,1}, {0,1}, {0,0}, {1,1}, {0,1}, {0,0}, {1,0}, {1,0}, {0,1}, {0,1}, {0,0}, {1,0}},
{{1,0}, {1,0}, {0,1}, {0,1}, {1,1}, {1,1}, {0,0}, {0,1}, {1,0}, {0,1}, {1,1}, {0,1}, {0,0}, {1,0}},
{{1,0}, {1,0}, {0,1}, {1,0}, {0,0}, {1,0}, {1,1}, {0,1}, {1,1}, {0,0}, {1,0}, {0,1}, {0,1}, {1,0}},
{{1,0}, {0,1}, {1,0}, {1,0}, {1,0}, {0,0}, {0,0}, {1,0}, {1,0}, {1,1}, {1,0}, {1,1}, {0,1}, {1,0}},
{{1,0}, {1,0}, {0,0}, {1,0}, {0,1}, {0,1}, {1,1}, {1,0}, {0,0}, {1,0}, {1,0}, {0,0}, {1,0}, {1,0}},
{{1,0}, {0,1}, {0,1}, {0,1}, {1,0}, {1,0}, {0,0}, {1,1}, {0,1}, {0,0}, {0,1}, {0,1}, {0,0}, {1,0}},
{{1,0}, {0,1}, {1,1}, {0,0}, {1,0}, {0,1}, {0,1}, {0,1}, {1,0}, {0,1}, {0,1}, {0,1}, {1,1}, {1,0}},
{{1,0}, {1,0}, {0,0}, {1,1}, {0,1}, {0,1}, {0,1}, {0,0}, {0,0}, {1,1}, {0,0}, {1,0}, {0,0}, {1,0}},
{{0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,0}}
};

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */ 
  
  /* screen size definition */
  size(800, 700);
  
  /* set font type and size */
  f = createFont("Arial", 12, true);

  
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

  for (int i = 0; i < cols; i++) // C = 17
  {
    for (int j = 0; j < rows; j++) //R = 14
    {
      h = maze[i][j][0] == 1 ? 0.9 : 0;
      v = maze[i][j][1] == 1 ? 0.9 : 0;      

        if (h != 0)
        {
          fboxes_h[j][i] = createWall(0.1 + h, 0.1, 1.8 + 1 * i, 15.5 - j);
          world.add(fboxes_h[j][i]);
        }
        if (v != 0)
        {
          fboxes_v[j][i] = createWall(0.1, 0.1 + v, 1.75 + 1 * i - (0.4), 15.5 - j - 0.4);
          world.add(fboxes_v[j][i]);
        }
    }
  }
  
  /* Start Button */
  c1                  = new FCircle(1); // diameter is 2
  c1.setPosition(edgeTopLeftX+1.5, edgeTopLeftY+15.2);
  c1.setFill(0, 255, 0);
  c1.setSensor(true);
  c1.setStaticBody(true);
  c1.setGrabbable(false);
  world.add(c1);
  
  /* Finish Button */
  c2                  = new FCircle(0.85);
  c2.setPosition(worldWidth-3.2, edgeTopLeftY+3);
  c2.setFill(200,0,0);
  c2.setStaticBody(true);
  c2.setSensor(true);
  c2.setGrabbable(false);
  world.add(c2);

 
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), 0);//gravityAcceleration); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  //world.setEdgesRestitution(.4);
  //world.setEdgesFriction(0.5);
 
  world.draw();
    
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/

public void addEnemy(FCircle obj, float xpos, float ypos, float xsp, float ysp)
{
  obj.attachImage(loadImage("../img/ghost.png")); 
  obj.setPosition(xpos, ypos);
  obj.setFill(128, 255, 0);
  //obj.setRotatable(true);
  obj.setStatic(false);
  //blob1.setSensor(true);
  obj.setVelocity(xsp, ysp);
  obj.setRestitution(1.0);
  obj.setDamping(0.0);
  obj.setGrabbable(false);
  world.add(obj);  
}

boolean render_enemies = false;

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
    
    if(gameStart)
    {
      if (!render_enemies)
      {
        addEnemy(blob1, 16.8, 8.0, 3.0, 0.0);
        addEnemy(blob2, 3.8, 5.0, 0.0, 6.0);
        addEnemy(blob3, 6.8, 7.0, 0.0, 2.0);
        addEnemy(blob4, 12.8, 15.0, 10.0, 0.0);
        addEnemy(blob5, 1.8, 11.0, 0.0, 5.0);
        addEnemy(blob6, 10.85, 9.0, 0.0, 3.5);
        addEnemy(blob7, 16.8, 5.0, 3.0, 0.0);
        render_enemies = true;
      }
      
      fill(0, 0, 0);
      //textAlign(CENTER);
      //text("Move the ball to the red circle", width/2, 60);
      textAlign(CENTER);
      //text("Touch the green circle to reset", width/2, 9);
      
      for (int i = 0; i < cols; i++)
      {
        for (int j = 0; j < rows; j++)
        {
          if (isValidHorz(i, j))
            fboxes_h[j][i].setFill(0, 0, 0);
          if (isValidVert(i, j))  
            fboxes_v[j][i].setFill(0, 0, 0);
        }
      }
    }
    
    else
    {
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch the green circle to start the maze, then touch the red circle to win", width/2, 60);
      
      for (int i = 0; i < cols; i++)
      {
        for (int j = 0; j < rows; j++)
        {
          if (isValidHorz(i, j))
            fboxes_h[j][i].setFill(255, 255, 255);
          if (isValidVert(i, j))  
            fboxes_v[j][i].setFill(255, 255, 255);
        }
      }
    } 
    //try {
    world.draw();
    //}
    //catch () {}
  }
}

public boolean isValidVert(int i, int j)
{
   return maze[i][j][1] == 0 ? false : true;
}

public boolean isValidHorz(int i, int j)
{
   return maze[i][j][0] == 0 ? false : true;
}

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
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
   if (s.h_avatar.isTouchingBody(c1)){
        gameStart = true;
        s.h_avatar.setSensor(false);
      }
      
      if (s.h_avatar.isTouchingBody(c2)) {
        gameStart = false;
        s.h_avatar.setSensor(true);
        clean_stage();
      }
      
      if (s.h_avatar.isTouchingBody(blob1) || s.h_avatar.isTouchingBody(blob2) || s.h_avatar.isTouchingBody(blob3)
      || s.h_avatar.isTouchingBody(blob4) || s.h_avatar.isTouchingBody(blob5) || s.h_avatar.isTouchingBody(blob6)) 
      {
        gameStart = false;
        s.h_avatar.setSensor(true);
        clean_stage();
        //s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2);
      }

    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}

public void clean_stage()
{
        render_enemies = false;
        world.remove(blob1);
        world.remove(blob2);
        world.remove(blob3);
        world.remove(blob4);
        world.remove(blob5);
        world.remove(blob6);
        world.remove(blob7);  
}
/* end simulation section **********************************************************************************************/

FBox createWall(float w, float h, float x, float y){
  FBox wall = new FBox(w,h);
  wall.setPosition(x, y);
  wall.setNoStroke();
  wall.setStatic(true);
  wall.setFill(204, 0, 102);
  return wall;
}
