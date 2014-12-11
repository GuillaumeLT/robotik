#include "Aria.h"

#define pi 3.141592654
#define kx 5
#define ky 500
#define kth 100
#define Cycle 20
#define Te Cycle*1e-3
#define a 0.1                      //vitesse de trj désirée

/**
  trajectoire 1 = droite
  version du 15/11/12
*/

class PrintingTask
{
public:
  // Constructor. Adds our 'user task' to the given robot object.
  PrintingTask(ArRobot *robot);

  // Destructor. Does nothing.
  ~PrintingTask(void) {}

  // This method will be called by the callback functor
  void doTask(void);
protected:
  ArRobot *myRobot;
  FILE *file;
  int i,j;         // indices utiles

  // The functor to add to the robot for our 'user task'.
  ArFunctorC<PrintingTask> myTaskCB;
};


// the constructor (note how it uses chaining to initialize myTaskCB)
PrintingTask::PrintingTask(ArRobot *robot) :
  myTaskCB(this, &PrintingTask::doTask)
{
  myRobot = robot;
  // just add it to the robot
  myRobot->addSensorInterpTask("PrintingTask", 50, &myTaskCB);
  file = ArUtil::fopen("data.dat", "w+");
  i=0;
  j=0;
}

#define square_len 6
double const x_final [4] = { square_len, square_len, 0, 0 };
double const y_final [4] = { 0, square_len, square_len, 0 };
double xd = 5;//x_final[0];
double yd = 0;//y_final[0];

void PrintingTask::doTask(void)
{
  static bool collision_mode = false;
  static int square_state = 0;
  static unsigned int collision_iter_stop = 0;
  double t = i++ * Te;

  // get the robot coordinate
  double x   = myRobot->getX() * 1e-3;
  static double x0 = x;
  double y   = myRobot->getY() * 1e-3;
  static double y0 = y;
  double th  = myRobot->getTh() * (pi / 180);

  // get the coordinate of the second point
  double l = 1;
  double x1 = x + l * cos(th);
  double y1 = y + l * sin(th);

  /* // desired */
  double xd = x_final[square_state];
  double yd = y_final[square_state];

  printf("xd:%f, yd:%f, x1:%f, y1:%f\n", xd, yd, x1, y1);
  if (abs(xd - x1) < 0.5 && abs(yd - y1) < 0.5)
  {
    printf("AUGMENTAGE\n");
    square_state = (square_state + 1) % 4;
  }

  if (!collision_mode)
  {
    if (
        myRobot->getSonarRange(4) < 500
        || myRobot->getSonarRange(5) < 500
       )
    {
      collision_mode = true;
      collision_iter_stop = i + 50;
    }
  }
  if (i == collision_iter_stop)
  {
    collision_mode = false;
    xd = x_final[square_state];
    yd = x_final[square_state];
  }

  /* // derivate */
  double xdp = 0;
  double ydp = 0;

  double lambda = 0.3;
  // z matrix
  double zx = xdp + lambda * (xd - x1);
  double zy = ydp + lambda * (yd - y1);

  /* //calcul des consignes */
  // J_1 = [ cos(theta),    sin(theta);
  //         -sin(theta)/l, cos(theta)/l]
  double v   = zx * cos(th) + zy * sin(th);
  double w   = -zx * sin(th) / l + zy * cos(th) / l;

  // megamaneuvre
  if (collision_mode)
  {
    v = -0.5;
    w = -0.8;
  }

  /* // envoi des consignes */
  myRobot->lock();
  myRobot->setRotVel(w * (180 / pi));
  myRobot->setVel(v * 1000);
  myRobot->unlock();

  /* // debogage */
  /* fprintf(file, "%f, %f, %f, %f, %f, %f %f %f\n", x,y,th,xd,yd,t); */
  /* printf("x:%f y:%f th:%f coli:%d v:%f w:%f stop:%d i:%d zx:%f zy:%d\n", */
  /*        myRobot->getX(), myRobot->getY(), */
  /*        myRobot->getTh(), collision_mode, */
  /*        v, w, collision_iter_stop, i, zx, zy); */

}

int main(int argc, char** argv)
{
  // the connection
  /* ArSimpleConnector con(&argc, argv); */
  /* if(!con.parseArgs()) */
  /*   { */
  /*     con.logOptions(); */
  /*     return 1; */
  /*   } */

  // robot
  ArRobot robot;

  // sonar array range device
  ArSonarDevice sonar;

  // This object encapsulates the task we want to do every cycle.
  // Upon creation, it puts a callback functor in the ArRobot object
  // as a 'user task'.
  PrintingTask pt(&robot);

  // the actions we will use to wander
  ArActionStallRecover recover;
  ArActionAvoidFront avoidFront;
  ArActionConstantVelocity constantVelocity("Constant Velocity", 400);

  // initialize aria
  Aria::init();

  // add the sonar object to the robot
  robot.addRangeDevice(&sonar);

  // open the connection to the robot; if this fails exit
  /* if(!con.connectRobot(&robot)) */
  /*   { */
  /*     printf("Could not connect to the robot.\n"); */
  /*     return 2; */
  /*   } */
  ArTcpConnection con;
  con.setPort("192.168.56.1", 8101);
  if (!con.openSimple())
  {
    printf("Open failed.");
    Aria::shutdown();
    return 1;
  }
  robot.setDeviceConnection(&con);
  if (!robot.blockingConnect())
  {
    printf("Could not connect to robot... Exiting.");
    Aria::shutdown();
    return 1;
  }
  printf("Connected to the robot. (Press Ctrl-C to exit)\n");


  // turn on the motors, turn off amigobot sounds
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUNDTOG, 0);

  // add the wander actions
  /*robot.addAction(&recover, 100);
  robot.addAction(&avoidFront, 50);
  robot.addAction(&constantVelocity, 25);
  */
  robot.setCycleTime(Cycle);
  // Start the robot process cycle running. Each cycle, it calls the robot's
  // tasks. When the PrintingTask was created above, it added a new
  // task to the robot. 'true' means that if the robot connection
  // is lost, then ArRobot's processing cycle ends and this call returns.
  robot.run(true);

  printf("Disconnected. Goodbye.\n");

  return 0;
}
