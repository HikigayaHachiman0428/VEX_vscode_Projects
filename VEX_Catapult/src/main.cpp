#include "tasks.h"
#include "autonomous.h"
#include "auto_path.h"
using namespace vex;
using namespace std;

void autonomous(void)
{
  manual = 0;
  switch (autoRoutine)
  {
  case 0:
    break;
  case 1:
    one();
    break;
  case 2:
    two();
    break;
  case 3:
    three();
    break;
  }
}

void usercontrol(void)
{
  bool lastL1 = 0, lastL2 = 0, lastR1 = 0, lastR2 = 0, lastLEFT = 0, lastRIGHT = 0;
  manual = 1;
  AutoCataInterrupt = 0;
  lck = 1;
  lckReset = 1;
  ch_state = 1;
  rota_0 = 0.6;
  rota_1 = 0.3;
  while (1)
  {
    if (BX && debug)
    {
    }
    if (LEFT && !lastLEFT)
      autoRoutine = autoRoutine == 0 ? 3 : autoRoutine - 1;
    if (RIGHT && !lastRIGHT)
      autoRoutine = autoRoutine == 3 ? 0 : autoRoutine + 1;
    if (manual)
    {
      if ((L1 && L2 && !lastL1) || (L1 && L2 && !lastL2))
      {
        manual = 0;
        autoCata = 1;
        lck = 0;
      }
      if (L1 && (R2 || R1))
      {
        lck = 0;
      }
      if (!lck)
      {
        cata(100 * L1 * (R2 - R1));
      }
      if ((!L1 && (R1 || R2) && lastL1) || (L1 && !R1 && lastR1) || (L1 && !R2 && lastR2))
      {
        lck = 1;
        lckReset = 1;
      }
    }
    else
    {
      if ((L1 && L2 && !lastL1) || (L1 && L2 && !lastL2))
      {
        // AutoCataInterrupt = 1;
      }
    }
    // ch_state = abs(Ch1)<=100? 1 : 0;
    ch_state = L2 ? 1 : 0;
    // if(fabs(Ch1)<0.7){
    //   ch_state = 1;
    // }else{
    //   ch_state = 0;
    // }

    Ch();
    if (CataReady)
    {
      intake(-100 * (R2 - R1));
    }
    else
    {
      intake(-100 * !L1 * (R2 - R1));
    }

    // index(100*L2*(R2-R1));

    // Brain.Screen.printAt(10, 60, "lck: %d", lck);
    // Brain.Screen.printAt(10, 80, "lckreset: %d", lckReset);
    // Brain.Screen.printAt(10, 100, "L2: %d", L2);
    // Brain.Screen.printAt(10, 120, "lastL2: %d", lastL2);

    lastL1 = L1;
    lastL2 = L2;
    lastR1 = R1;
    lastR2 = R2;
    lastLEFT = LEFT;
    lastRIGHT = RIGHT;
    delay(10);
  }
}
int printat()
{
  while (1)
  {
    Brain.Screen.printAt(10, 160, "autoRoutine: %d", autoRoutine);
    Brain.Screen.printAt(10, 180, "Distance: %f", getDis);
    Brain.Screen.printAt(10, 200, "targetDistance: %f", targetDis);
    delay(10);
  }
}

int printInfo()
{
  while (1)
  {
    bool GYRO = 1, VOLTAGE = 1, ENCODER = 1, AUTO = 0;
    Controller1.Screen.setCursor(1, 1);
    if (GYRO)
    {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("Heading: ");
      Controller1.Screen.print(getHeading());
      Controller1.Screen.newLine();
    }
    if (VOLTAGE)
    {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("Voltage: ");
      Controller1.Screen.print(Brain.Battery.capacity());
      Controller1.Screen.newLine();
    }
    if (ENCODER)
    {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("ForwardEncoder: ");
      Controller1.Screen.print(getForwardEncoder());
      Controller1.Screen.newLine();
    }
    if (AUTO)
    {
      Controller1.Screen.clearLine();
      Controller1.Screen.print("AUTOROUTINE: ");
      Controller1.Screen.print(autoRoutine);
      Controller1.Screen.newLine();
    }
  }
  return 1;
}

int main()
{
  delay(200);
  task LCKON(LCK);
  task AUTOCATA(AutoCata);
  task ADJUST(adjustDis);
  Controller1.Screen.clearScreen();
  auto timer = MyTimer();
  while (Gyro.isCalibrating())
  {
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("CALIBRATING... DO NOT MOVE");
  }
  while (timer.getTime() < 50000)
    delay(1);
  task GyroSensor(gyroSensor);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  Controller1.Screen.clearScreen();
  task BRAIN_PRINT(printat);
  task CONTROL_PRINT(printInfo);
  delay(200);
  while (1)
  {
    delay(100);
  }
}
