#include "Aria.h"

#define pi 3.141592654
#define kx 5
#define ky 500
#define kth 100
#define Cycle 20
#define Te Cycle*1e-3
#define a 0.1

#define LOCAL_MODE
/* #define DEBUG */

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

#define SQUARE_LEN 6
double const x_final [4] = { SQUARE_LEN, SQUARE_LEN, 0, 0 };
double const y_final [4] = { 0, SQUARE_LEN, SQUARE_LEN, 0 };
double xd = x_final[0];
double yd = y_final[0];

void PrintingTask::doTask(void)
{
  static int collision_mode = 0;                // Etape de l'evitement
  static unsigned int collision_iter_stop = 0;  // Utilise pour les evitements
  static int square_state = 0;                  // Point destination du carre
  i++;

  // infos robot
  double x = myRobot->getX() * 1e-3;
  double y = myRobot->getY() * 1e-3;
  double th  = myRobot->getTh() * (pi / 180);

  // second point du robot
  double l = 0.5;
  double x1 = x + l*cos(th);
  double y1 = y + l*sin(th);

  // destination
  double xd = x_final[square_state];
  double yd = y_final[square_state];

  // matrice z
  double zx = 0.3*(xd - x1);
  double zy = 0.3*(yd - y1);

  // consignes
  double v = zx * cos(th) + zy * sin(th);
  double w = -zx * sin(th) / l + zy * cos(th) / l;

  // detection d'obstacle et evitement
  if (abs(xd - x1) < 0.5 && abs(yd - y1) < 0.5)
  {
    square_state = (square_state + 1) % 4;
  }

  if (collision_mode == 0)
  {
    if (myRobot->getSonarRange(3) < 500
        || myRobot->getSonarRange(4) < 500
        || myRobot->getSonarRange(5) < 500
        || myRobot->getSonarRange(6) < 500)
    {
      collision_mode = 1;
      collision_iter_stop = i + 40;
    }
  }
  if (collision_iter_stop == i)
  {
    if (collision_mode == 1)
      collision_iter_stop = i + 80;
    if (collision_mode == 2)
      collision_iter_stop = i + 45;
    if (collision_mode == 3)
    {
      xd = x_final[square_state];
      yd = x_final[square_state];
    }
    collision_mode = (collision_mode + 1) % 4;
  }

  // maneuvre d'evitement
  if (collision_mode == 1)
  {
    v = -0.1;
    w = -1;
  }
  if (collision_mode == 2)
  {
    v = 1;
    w = 0;
  }
  if (collision_mode == 3)
  {
    v = 0;
    w = 1;
  }

  // envoi des consignes
  myRobot->lock();
  myRobot->setRotVel(w * (180 / pi));
  myRobot->setVel(v * 1000);
  myRobot->unlock();

#ifdef DEBUG
  printf("x:%f y:%f th:%f coli:%d v:%f w:%f stop:%d i:%d zx:%f zy:%d\n",
         myRobot->getX(), myRobot->getY(),
         myRobot->getTh(), collision_mode,
         v, w, collision_iter_stop, i, zx, zy);
#endif
}

int main(int argc, char** argv)
{
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

#ifdef LOCAL_MODE
  // the connection
  ArSimpleConnector con(&argc, argv);
  if(!con.parseArgs())
    {
      con.logOptions();
      return 1;
    }

  // open the connection to the robot; if this fails exit
  if(!con.connectRobot(&robot))
    {
      printf("Could not connect to the robot.\n");
      return 2;
    }
#endif

#ifndef LOCAL_MODE
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
#endif

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
