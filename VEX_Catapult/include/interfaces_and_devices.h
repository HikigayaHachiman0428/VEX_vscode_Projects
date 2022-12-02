#ifndef API_H
#define API_H

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

brain Brain;
controller Controller1 = controller(primary);
competition Competition;

#ifdef _16950c
// motor LA = motor(PORT11, ratio18_1, 1);
motor LB = motor(PORT11, ratio18_1, 1);
motor LC = motor(PORT17, ratio18_1, 1);
// motor RA = motor(PORT18, ratio18_1, 1);
motor RB = motor(PORT18, ratio18_1, 1);
motor RC = motor(PORT12, ratio18_1, 1);

motor itk = motor(PORT1, ratio18_1, 0);
motor cat = motor(PORT19, ratio18_1, 0);
motor cat2 = motor(PORT20, ratio18_1, 0);
motor roll1 = motor(PORT1, ratio18_1, 0);
motor expand = motor(PORT10, ratio18_1, 1);
vex::distance dis = vex::distance(PORT15);
inertial Gyro = inertial(PORT9);
optical opt = optical(PORT10);
#endif

#ifdef _11x
// motor LA = motor(PORT11, ratio18_1, 1);
motor LB = motor(PORT1, ratio18_1, 1);
motor LC = motor(PORT19, ratio18_1, 1);
// motor RA = motor(PORT18, ratio18_1, 1);
motor RB = motor(PORT16, ratio18_1, 1);
motor RC = motor(PORT18, ratio18_1, 1);

motor itk = motor(PORT5, ratio18_1, 0);
motor cat = motor(PORT20, ratio18_1, 0);
motor cat2 = motor(PORT15, ratio18_1, 0);
motor roll1 = motor(PORT5, ratio18_1, 0);
motor expand = motor(PORT10, ratio18_1, 1);
vex::distance dis = vex::distance(PORT17);
inertial Gyro = inertial(PORT3);
optical opt = optical(PORT6);
#endif

#endif