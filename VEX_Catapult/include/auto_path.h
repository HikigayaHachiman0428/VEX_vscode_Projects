#ifndef _AUTO_PATH
#define _AUTO_PATH
#include "autonomous.h"
using namespace vex;

#ifdef _11x
// PID
void PIDGyroTurn(float target)
{
    // resetGyro();
    auto pid = PID();
    auto myTimer = MyTimer();
    // 觉得慢了，调第一个；觉得冲不动了，调第二个；觉得冲过头了，调大第三个
    pid.setCoefficient(3.1, 0.5, 30);
    pid.setTarget(target);
    pid.setIMax(IMAX);
    pid.setIRange(60); // 60
    pid.setErrorTolerance(1);
    pid.setDTolerance(4); // 2.6 deg / sec
    pid.setJumpTime(0.01);
    while (!pid.targetArrived())
    {
        pid.update(getHeading());
        driveRotate(pid.getOutput());
        delay(10);
    }
    driveRotate(0);
}

void PIDForward(float target, float vmax = 150)
{
    resetChassisEncoder();
    float heading = getHeading();
    float dt = 0.01;
    float timeroffset = TIMER;
    float timeused = 0;
    float kp = 1.0;     // 3.0 , 3.95
    float ki = 12.0;    // 28.9 , 28.6
    float kd = 0.1;     // 29.9 , 29.9
    float irange = 2.0; // ji fen fan wei
    float istart = 60;  // start to integral
    float dtol = 0.15;
    float errortolerance = 15; // 2.5zd
    float lim = vmax;
    float error = target - getForwardEncoder();
    float lasterror;
    float v = 0;
    float i = 0;
    bool arrived;
    float timetol = abbs(error) < 20 ? 700 : abbs(error) * 24;
    float pow;
    lasterror = error;
    arrived = error == 0;
    while (!arrived)
    {
        timeused = TIMER - timeroffset;
        error = abbs(target) - getForwardEncoder();
        v = (error - lasterror) / dt;
        if ((abbs(error) < errortolerance && abbs(v) <= dtol) ||
            timeused > timetol)
        {
            arrived = true;
        }
        if (abbs(i) < irange && abbs(error) < istart)
            i += sgn(error) * dt;
        if (error * lasterror <= 0)
            i = 0;
        pow = kp * error + kd * v + ki * i;
        pow = abbs(pow) > lim ? sgn(pow) * lim : pow;
        // driveForward(sgn(target) * pow);
        driveForward(sgn(target) * pow, heading);
        cout << error << "  " << pow << "  " << endl;
        // cout<<gyroValue()<<endl;
        // cout<<timeused<<endl;
        // printScreen(10,100,"Iner %f",Iner.rotation());
        lasterror = error;
        delay(10);
    }
    driveForward(0);
}
// 自动线路
void test()
{
    // autoCata = 1;
    intake(100);
    PIDForward(1000, 45);
    intake(0);
}

void one() // 必须拿下
{
    auto time = MyTimer();
    resetGyro();
    autoCata = 1;
    // 预装
    PIDGyroTurn(108);
    PIDForward(-150);
    shoot();
    delay(800);
    intake(100);
    delay(1000);
    intake(0);
    shoot();
    // delay(100);
    PIDGyroTurn(45);
    GyroTimerForward(100, 1500, 90);
    Rroll();

    PIDForward(-55);
    PIDGyroTurn(225);

    intake(100);
    PIDForward(800, 55);

    // 第一个
    delay(100);
    PIDGyroTurn(116);
    intake(0);
    shoot();

    // 第二个
    PIDGyroTurn(225);
    intake(100);
    PIDForward(330, 55);
    delay(100);
    intake(0);
    PIDGyroTurn(125);
    shoot();

    while (time.getTime() < 15 * 60 * 1000 - 10)
    {
        expand.spin(reverse, 120 * 60, voltageUnits::mV);
        delay(1);
    }
    expand.spin(reverse, 0, voltageUnits::mV);
}

void two() // double sb
{
    auto time = MyTimer();
    resetGyro();
    addGyroBias(-180);
    farmode = 1;
    Rroll();
    autoCata = 1;
    PIDForward(-700);
    PIDGyroTurn(45);
    PIDForward(2263);
    PIDGyroTurn(135);
    shoot();
    intake(100);
    PIDForward(282);
    intake(0);
    PIDForward(-282);
    shoot();
    PIDGyroTurn(45);
    PIDForward(2263);
    PIDGyroTurn(90);
    timerForward(100, 300);
    Rroll();
    while (time.getTime() < 15 * 60 * 1000 - 10)
    {
        expand.spin(reverse, 120 * 60, voltageUnits::mV);
        delay(1);
    }
    expand.spin(reverse, 0, voltageUnits::mV);
    farmode = 0;
}
int three() // 一分钟技能赛
{
    resetGyro();
    autoCata = 1;
    // 1.	转滚筒，前进，吸入盘子，转弯，面向滚筒
    Rroll(0);
    // delay(100);
    // timerForward(50, 300);
    // delay(100);
    PIDForward(-37);
    // delay(100);
    intake(100);
    PIDGyroTurn(155);
    // delay(150);
    // intake(100);
    PIDForward(450);
    // delay(150);
    PIDGyroTurn(88);
    intake(0);
    // delay(100);
    //  2.	前进至滚筒处，转滚筒
    timerForward(60, 700);
    // delay(150);
    Rroll(0);
    // 3.	弧线前进至可以投High Goal的位置，投
    // delay(100);
    // timerForward(50, 400);
    // delay(100);
    PIDForward(-35);
    // delay(100);
    PIDGyroTurn(0); // 7.5
    // delay(150);
    PIDForward(-1550); // 1565
    // delay(100);
    // PIDGyroTurn(12.5);
    // delay(100);
    shoot();
    delay(400);
    // PIDGyroTurn(7.5);
    // delay(100);
    //
    //  4. 后退至disc斜线处，转弯面向disc（-135）
    PIDForward(1310); // 1385
    // delay(100);
    PIDGyroTurn(-135.5); //-132.5
    // delay(150);
    //  5. 前进，吸掉，转弯平行于边界条（45），后退，转弯面向high goal（90），投
    intake(100);
    PIDForward(1575, 50);
    // delay(100);
    PIDForward(-200);
    // delay(100);
    PIDGyroTurn(-90); // 41
    // delay(100);
    intake(-100);
    PIDForward(-1100);
    intake(0);
    // delay(500);
    PIDGyroTurn(4.5);
    // delay(150);
    PIDForward(-450); // 400
    // delay(150);
    // PIDGyroTurn(5.5);
    // delay(150);
    shoot();
    delay(400);
    // PIDGyroTurn(0.5);
    // delay(100);
    PIDForward(570); // 520
    // delay(100);
    //  6.   转弯（-90），前进到斜线，转弯面向disc堆（-45），前进吃掉3个disc，转弯（45），后退到可投位置，投
    PIDGyroTurn(-90);
    // delay(150);
    PIDForward(900); // 900
    // delay(100);
    PIDGyroTurn(-133); // 46
    // delay(100);
    intake(100);
    PIDForward(1800); // 1500
    // delay(100);
    intake(0);
    // 7.
    PIDGyroTurn(-180);
    delay(100);
    timerForward(30, 500); // 60
    delay(100);
    Rroll(0);
    delay(100);
    timerForward(60, 300);
    delay(100);
    PIDForward(-60);
    delay(100);
    PIDGyroTurn(-1.5);
    delay(100);
    PIDForward(-1610);
    delay(100);
    PIDGyroTurn(-16.5);
    delay(200);
    shoot();
    delay(300);
    PIDGyroTurn(-1.5);
    delay(100);
    PIDForward(670);
    delay(100);
    PIDGyroTurn(88.5);
    delay(100);
    PIDForward(630);
    delay(100);
    intake(0);
    PIDGyroTurn(-2.5);
    delay(100);
    intake(100);
    timerForward(45, 3000);
    delay(50);
    intake(0);
    Rroll(0);
    delay(50);
    timerForward(30, 500);
    delay(50);
    PIDForward(-60);
    delay(100);
    PIDGyroTurn(-92.5);
    delay(150);
    PIDForward(-1560); // 1565
    delay(100);
    PIDGyroTurn(-83.5);
    delay(100);
    shoot();
    delay(400);
    PIDGyroTurn(-92.5);
    delay(100);
    PIDForward(1520);
    delay(100);
    PIDGyroTurn(-136.5);
    delay(100);
    intake(100);
    PIDForward(470);
    intake(0);
    return 1;
}
#endif

#ifdef _16950c
// PID
void PIDGyroTurn(float target)
{
    // resetGyro();
    auto pid = PID();
    auto myTimer = MyTimer();
    // 觉得慢了，调第一个；觉得冲不动了，调第二个；觉得冲过头了，调大第三个
    pid.setCoefficient(3.1, 0.5, 30);
    pid.setTarget(target);
    pid.setIMax(IMAX);
    pid.setIRange(60); // 60
    pid.setErrorTolerance(1);
    pid.setDTolerance(4); // 2.6 deg / sec
    pid.setJumpTime(0.01);
    while (!pid.targetArrived())
    {
        pid.update(getHeading());
        driveRotate(pid.getOutput());
        delay(10);
    }
    driveRotate(0);
}

void PIDForward(float target, float vmax = 150)
{
    resetChassisEncoder();
    float heading = getHeading();
    float dt = 0.01;
    float timeroffset = TIMER;
    float timeused = 0;
    float kp = 1.0;     // 3.0 , 3.95
    float ki = 12.0;    // 28.9 , 28.6
    float kd = 0.1;     // 29.9 , 29.9
    float irange = 2.0; // ji fen fan wei
    float istart = 60;  // start to integral
    float dtol = 0.15;
    float errortolerance = 15; // 2.5zd
    float lim = vmax;
    float error = target - getForwardEncoder();
    float lasterror;
    float v = 0;
    float i = 0;
    bool arrived;
    float timetol = abbs(error) < 20 ? 700 : abbs(error) * 24;
    float pow;
    lasterror = error;
    arrived = error == 0;
    while (!arrived)
    {
        timeused = TIMER - timeroffset;
        error = abbs(target) - getForwardEncoder();
        v = (error - lasterror) / dt;
        if ((abbs(error) < errortolerance && abbs(v) <= dtol) ||
            timeused > timetol)
        {
            arrived = true;
        }
        if (abbs(i) < irange && abbs(error) < istart)
            i += sgn(error) * dt;
        if (error * lasterror <= 0)
            i = 0;
        pow = kp * error + kd * v + ki * i;
        pow = abbs(pow) > lim ? sgn(pow) * lim : pow;
        // driveForward(sgn(target) * pow);
        driveForward(sgn(target) * pow, heading);
        cout << error << "  " << pow << "  " << endl;
        // cout<<gyroValue()<<endl;
        // cout<<timeused<<endl;
        // printScreen(10,100,"Iner %f",Iner.rotation());
        lasterror = error;
        delay(10);
    }
    driveForward(0);
}
// 自动线路
void test()
{
    // autoCata = 1;
    intake(100);
    PIDForward(1000, 45);
    intake(0);
}

void one() // 必须拿下
{
    auto time = MyTimer();
    resetGyro();
    // farmode = 1;
    autoCata = 1;
    PIDForward(540);
    // delay(100);
    PIDGyroTurn(90);
    timerForward(100, 200);
    Rroll();

    PIDForward(-55);
    PIDGyroTurn(225);
    // 预装
    PIDForward(470);
    PIDGyroTurn(108);
    // shoot();
    intake(100);
    delay(800);
    intake(0);
    // shoot();

    // 第一个
    PIDGyroTurn(225);
    intake(100);
    PIDForward(330, 55);
    delay(100);
    intake(0);
    PIDGyroTurn(116);
    // shoot();

    // 第二个
    PIDGyroTurn(225);
    intake(100);
    PIDForward(330, 55);
    delay(100);
    intake(0);
    PIDGyroTurn(125);
    // shoot();

    while (time.getTime() < 15 * 60 * 1000 - 10)
    {
        expand.spin(reverse, 120 * 60, voltageUnits::mV);
        delay(1);
    }
    expand.spin(reverse, 0, voltageUnits::mV);
    farmode = 0;
}

void two() // double sb
{
    auto time = MyTimer();
    resetGyro();
    addGyroBias(-180);
    farmode = 1;
    Rroll();
    autoCata = 1;
    PIDForward(-700);
    PIDGyroTurn(45);
    PIDForward(2263);
    PIDGyroTurn(135);
    shoot();
    intake(100);
    PIDForward(282);
    intake(0);
    PIDForward(-282);
    shoot();
    PIDGyroTurn(45);
    PIDForward(2263);
    PIDGyroTurn(90);
    timerForward(100, 300);
    Rroll();
    while (time.getTime() < 15 * 60 * 1000 - 10)
    {
        expand.spin(reverse, 120 * 60, voltageUnits::mV);
        delay(1);
    }
    expand.spin(reverse, 0, voltageUnits::mV);
    farmode = 0;
}
int three() // 一分钟技能赛
{
    farmode = 0;
    resetGyro();
    // autoCata = 1;
    //  1.	转滚筒，前进，吸入盘子，转弯，面向滚筒
    timerForward(30, 100);
    driveForward(10);
    timerIndex(-100, 150);

    // delay(100);
    // timerForward(50, 300);
    // delay(100);
    // PIDForward(-37);
    timerForward(-40, 80);
    delay(100);
    intake(100);
    PIDGyroTurn(155);
    // delay(150);
    // intake(100);
    PIDForward(550);
    // delay(150);
    PIDGyroTurn(88);
    intake(0);
    // delay(100);
    //  2.	前进至滚筒处，转滚筒
    timerForward(60, 700);
    // delay(150);
    driveForward(10);
    timerIndex(-100, 150);
    // 3.	弧线前进至可以投High Goal的位置，投
    // delay(100);
    // timerForward(50, 400);
    // delay(100);
    timerForward(-40, 80);
    // delay(100);
    PIDGyroTurn(0); // 7.5
    // delay(150);
    EncoderForward(-100, 1000);
    LB.resetPosition();
    shoot();
    delay(200);
    int s = LB.position(deg);
    //   4. 后退至disc斜线处，转弯面向disc（-135）
    PIDForward(900 + s); // 1385
    // delay(100);
    PIDGyroTurn(-45); //-132.5
    // delay(150);
    //  5. 前进，吸掉，转弯平行于边界条（45），后退，转弯面向high goal（90），投
    intake(100);
    EncoderForward(100, 200);
    PIDForward(1000, 50);
    delay(400);
    // PIDGyroTurn(-);
    // timerJumpForward(-100,500);
    //  delay(100);
    PIDForward(-1300);
    // delay(100);
    PIDGyroTurn(0); // 41
    EncoderForward(-100, 400);
    LB.resetPosition();
    shoot();
    delay(200);
    s = LB.position(deg);
    PIDForward(s + 200);
    PIDGyroTurn(-45);
    intake(100);
    PIDForward(800, 60);
    delay(30000);

    // delay(100);
    intake(-100);
    PIDForward(-1100);
    intake(0);
    // delay(500);
    PIDGyroTurn(4.5);
    // delay(150);
    PIDForward(-450); // 400
    // delay(150);
    // PIDGyroTurn(5.5);
    // delay(150);
    shoot();
    delay(400);
    // PIDGyroTurn(0.5);
    // delay(100);
    PIDForward(570); // 520
    // delay(100);
    //  6.   转弯（-90），前进到斜线，转弯面向disc堆（-45），前进吃掉3个disc，转弯（45），后退到可投位置，投
    PIDGyroTurn(-90);
    // delay(150);
    PIDForward(900); // 900
    // delay(100);
    PIDGyroTurn(-133); // 46
    // delay(100);
    intake(100);
    PIDForward(1800); // 1500
    // delay(100);
    intake(0);
    // 7.
    PIDGyroTurn(-180);
    delay(100);
    timerForward(30, 500); // 60
    delay(100);
    Rroll(0);
    delay(100);
    timerForward(60, 300);
    delay(100);
    PIDForward(-60);
    delay(100);
    PIDGyroTurn(-1.5);
    delay(100);
    PIDForward(-1610);
    delay(100);
    PIDGyroTurn(-16.5);
    delay(200);
    shoot();
    delay(300);
    PIDGyroTurn(-1.5);
    delay(100);
    PIDForward(670);
    delay(100);
    PIDGyroTurn(88.5);
    delay(100);
    PIDForward(630);
    delay(100);
    intake(0);
    PIDGyroTurn(-2.5);
    delay(100);
    intake(100);
    timerForward(45, 3000);
    delay(50);
    intake(0);
    Rroll(0);
    delay(50);
    timerForward(30, 500);
    delay(50);
    PIDForward(-60);
    delay(100);
    PIDGyroTurn(-92.5);
    delay(150);
    PIDForward(-1560); // 1565
    delay(100);
    PIDGyroTurn(-83.5);
    delay(100);
    shoot();
    delay(400);
    PIDGyroTurn(-92.5);
    delay(100);
    PIDForward(1520);
    delay(100);
    PIDGyroTurn(-136.5);
    delay(100);
    intake(100);
    PIDForward(470);
    intake(0);
    return 1;
}
#endif

#endif