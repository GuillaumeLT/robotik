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

#include <iostream>
void PrintingTask::doTask(void)
{
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

  double r = 5;
  double wmax = 0.1;
  // desired
  double xd = 3;
  double yd = 3;

  // derivate
  double xdp = 0;
  double ydp = 0;

  double lambda = 1;
  // z matrix
  double zx = xdp + lambda * (xd - x1);
  double zy = ydp + lambda * (yd - y1);

  //calcul des consignes
  // J_1 = [ cos(theta),    sin(theta);
  //         -sin(theta)/l, cos(theta)/l]
  double v   = zx * cos(th) + zy * sin(th);
  double w   = -zx * sin(th) / l + zy * cos(th) / l;

  // envoi des consignes
  myRobot->lock();
  myRobot->setRotVel(w * (180 / pi));
  myRobot->setVel(v * 1000);
  myRobot->unlock();

  fprintf(file, "%f, %f, %f, %f, %f, %f %f %f\n", x,y,th,xd,yd,t);

  // Need sensor readings? Try myRobot->getRangeDevices() to get all
  // range devices, then for each device in the list, call lockDevice(),
  // getCurrentBuffer() to get a list of recent sensor reading positions, then
  // unlockDevice().
}

int main(int argc, char** argv)
{
  // the connection
  ArSimpleConnector con(&argc, argv);
  if(!con.parseArgs())
    {
      con.logOptions();
      return 1;
    }

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
  if(!con.connectRobot(&robot))
    {
      printf("Could not connect to the robot.\n");
      return 2;
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
