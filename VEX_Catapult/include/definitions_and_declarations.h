#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// #define _11x
#define _16950c

#include "interfaces_and_devices.h"
#include <iostream>
using namespace std;

#define TIMER Brain.timer(vex::timeUnits::msec)

#define Ch1 Controller1.Axis1.position(percent)
#define Ch2 Controller1.Axis2.position(percent)
#define Ch3 Controller1.Axis3.position(percent)
#define Ch4 Controller1.Axis4.position(percent)

#define BA Controller1.ButtonA.pressing()
#define BB Controller1.ButtonB.pressing()
#define BX Controller1.ButtonX.pressing()
#define BY Controller1.ButtonY.pressing()

#define L1 Controller1.ButtonL1.pressing()
#define L2 Controller1.ButtonL2.pressing()
#define R1 Controller1.ButtonR1.pressing()
#define R2 Controller1.ButtonR2.pressing()

#define UP Controller1.ButtonUp.pressing()
#define DOWN Controller1.ButtonDown.pressing()
#define LEFT Controller1.ButtonLeft.pressing()
#define RIGHT Controller1.ButtonRight.pressing()

#define SDFile "t.h"

#define getCataEncoder (cat.position(deg))
#define getLimitValue (lmt.value())
#define getLimit2Value (lmt2.value())
#define getDis (dis.objectDistance(mm))
#define isSDInserted (Brain.SDcard.isInserted())
#define isFileExists (Brain.SDcard.exists(SDFile))
#define sign(x) (x == 0 ? 0 : (x > 0 ? 1 : -1))
#define sgn(x) (x > 0 ? 1 : -1)
#define getGyro Gyro.rotation()

#define delay vexDelay

#define rollHue (opt.hue())
#define rollMid (rollHue < 10 || rollHue > 308)
#define rollBlue (rollHue <= 220 && rollHue >= 200)
#define rollRed (rollHue >= 10 && rollHue <= 25)
#define IMAX 30

static int ch_state = 0;
static bool lck = 1;
static bool lckReset = 0;
static bool autoCata = 0;
static bool manual = 1;
static bool AutoCataInterrupt = 0;
static int autoRoutine = 3;
static bool debug = 1;
static bool farmode = 0;
static float rota_0 = 0.5;
static float rota_1 = 0.3;
static float targetDis = 20;

#define CataReady ((getDis < targetDis) && (getDis > 0) && dis.installed())

float abbs(float x)
{
    return x >= 0 ? x : -x;
}

#endif