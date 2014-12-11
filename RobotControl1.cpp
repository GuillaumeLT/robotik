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

#define square_len 5
double const x_final [4] = { square_len, square_len, 0, 0 };
double const y_final [4] = { 0, square_len, square_len, 0 };
double xd = x_final[0];
double yd = y_final[0];

void PrintingTask::doTask(void)
{
  /* static bool collision_mode = false; */
  /* static int square_state = 0; */
  /* static unsigned int collision_iter_stop = 0; */
  /* double t = i++ * Te; */

  /* // get the robot coordinate */
  /* double x   = myRobot->getX() * 1e-3; */
  /* static double x0 = x; */
  /* double y   = myRobot->getY() * 1e-3; */
  /* static double y0 = y; */
  /* double th  = myRobot->getTh() * (pi / 180); */

  /* // get the coordinate of the second point */
  /* double l = 1; */
  /* double x1 = x + l * cos(th); */
  /* double y1 = y + l * sin(th); */

  /* double r = 5; */
  /* double wmax = 0.1; */

  /* // desired */
  /* double xd = 3; */
  /* double yd = 3; */

  /* if (!collision_mode) */
  /* { */
  /*   if ( */
  /*       myRobot->getSonarRange(4) < 500 */
  /*       || myRobot->getSonarRange(5) < 500 */
  /*      ) */
  /*   { */
  /*     collision_mode = true; */
  /*     collision_iter_stop = i + 50; */
  /*   } */
  /* } */
  /* if (i == collision_iter_stop) */
  /* { */
  /*   collision_mode = false; */
  /*   xd = x_final[square_state]; */
  /*   yd = x_final[square_state]; */
  /* } */

  /* // derivate */
  /* double xdp = 0; */
  /* double ydp = 0; */

  /* double lambda = 0.5; */
  /* // z matrix */
  /* double zx = xdp + lambda * (xd - x1); */
  /* double zy = ydp + lambda * (yd - y1); */

  /* //calcul des consignes */
  /* // J_1 = [ cos(theta),    sin(theta); */
  /* //         -sin(theta)/l, cos(theta)/l] */
  /* double v   = zx * cos(th) + zy * sin(th); */
  /* double w   = -zx * sin(th) / l + zy * cos(th) / l; */

  // megamaneuvre
  /* if (collision_mode) */
  /* { */
  /*   v = -0.5; */
  /*   w = 0.8; */
  /* } */
  /* else */
  /* { */
  /* } */

  /* // envoi des consignes */
  /* myRobot->lock(); */
  /* myRobot->setRotVel(w * (180 / pi)); */
  /* myRobot->setVel(v * 1000); */
  /* myRobot->unlock(); */

  /* // debogage */
  /* fprintf(file, "%f, %f, %f, %f, %f, %f %f %f\n", x,y,th,xd,yd,t); */
  /* printf("x:%f y:%f th:%f coli:%d v:%f w:%f stop:%d i:%d zx:%f zy:%d\n", */
  /*        myRobot->getX(), myRobot->getY(), */
  /*        myRobot->getTh(), collision_mode, */
  /*        v, w, collision_iter_stop, i, zx, zy); */

  double x, y, th, t;                      //variables de sortie
  double ex, ey, eth;                   //les erreurs
  double v, w;                         //variables de commande
  double vr, wr;                       //consignes de vitesse
  double l, xt, yt, tht;                 //variables dues aux consignes
  int c1, c2, c3, c4, c5, c6, c7, c8;

  wr = 0;
  vr = 0.05;
  //calcul des trajectoires désirées
  tht = pi / 6;
  t = i*Te;
  l = a*t;
  if (fmod(t, 20) < 5)
  {
    xt = 5 * t;
    yt = 0;
  }
  else if (fmod(t, 20) < 10)
  {
    xt = 0;
    yt = 5 * t;
  }
  else if (fmod(t, 20) < 15)
  {
    xt = -5 * t;
    yt = 0;
  }
  else if (fmod(t, 20) < 20)
  {
    xt = 0;
    yt = -5 * t;
  }
  i++;
  // récupérer some info about the robot
  x = myRobot->getX()*1e-3;
  y = myRobot->getY()*1e-3;
  th = myRobot->getTh()*(pi / 180);

  // calcul des erreurs
  ex = (xt - x)*cos(th) + (yt - y)*sin(th);
  ey = -(xt - x)*sin(th) + (yt - y)*cos(th);
  eth = tht - th;

  //calcul des consignes
  v = vr*cos(eth) + kx*ex;
  w = wr + vr*(ky*ey + kth*eth);

  // envoi des consignes
  myRobot->lock();
  myRobot->setRotVel(w*(180 / pi));
  myRobot->setVel(v * 1000);
  myRobot->unlock();

  fprintf(file, "%f, %f, %f, %f, %f, %f, %f %f %f\n", x, y, th, xt, yt, tht, t);

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
  /* ArTcpConnection con; */
  /* con.setPort("192.168.56.1", 8101); */
  /* if (!con.openSimple()) */
  /* { */
  /*   printf("Open failed."); */
  /*   Aria::shutdown(); */
  /*   return 1; */
  /* } */
  /* robot.setDeviceConnection(&con); */
  /* if (!robot.blockingConnect()) */
  /* { */
  /*   printf("Could not connect to robot... Exiting."); */
  /*   Aria::shutdown(); */
  /*   return 1; */
  /* } */
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
