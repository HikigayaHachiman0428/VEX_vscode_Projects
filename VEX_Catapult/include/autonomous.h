#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "definitions_and_declarations.h"
#include "interfaces_and_devices.h"
#include "tasks.h"
#include "GYRO.h"
#include "PID.h"
#include "TIMER.h"
#include <iostream>
float getHeading();
void driveForward(int v)
{
  v = v * 120;
  LB.spin(fwd, v, vex::voltageUnits::mV);
  LC.spin(fwd, v, vex::voltageUnits::mV);
  RB.spin(fwd, -v, vex::voltageUnits::mV);
  RC.spin(fwd, -v, vex::voltageUnits::mV);
}

void moveForwardRotate(float speed, float angle_)
{
  speed = 120 * speed;
  angle_ = 120 * angle_;
  LB.spin(fwd, speed + angle_, vex::voltageUnits::mV);
  LC.spin(fwd, speed + angle_, vex::voltageUnits::mV);
  RC.spin(fwd, -speed + angle_, vex::voltageUnits::mV);
  RB.spin(fwd, -speed + angle_, vex::voltageUnits::mV);
}

void driveForward(int v, float heading)
{
  float errorGyro;
  float errorGyroLimit = 30;
  float errorGyroTolerance = 2.5;
  float kpGyro = 5.8;
  errorGyro = abbs(heading - getHeading()) > errorGyroTolerance ? heading - getHeading() : 0;
  errorGyro = abbs(heading - getHeading()) > errorGyroLimit ? sgn(heading - getHeading()) * errorGyroLimit : heading - getHeading();
  moveForwardRotate(v, kpGyro * errorGyro);
}

void intakePlate(float v)
{
  itk.spin(fwd, v, vex::voltageUnits::mV);
}

void driveRotate(int v)
{
  v = v * 120;
  LB.spin(fwd, v, vex::voltageUnits::mV);
  LC.spin(fwd, v, vex::voltageUnits::mV);
  RB.spin(fwd, v, vex::voltageUnits::mV);
  RC.spin(fwd, v, vex::voltageUnits::mV);
}

void timerForward(int v, int t)
{
  float startTime = Brain.timer(msec);
  float heading = getHeading();
  while (Brain.timer(msec) - t < startTime)
  {
    driveForward(v);
  }
  driveForward(0);
}

void shoot();
void timerJumpForward(float velocity, float encoderTarget)
{
  LB.resetPosition();
  while (abbs(LB.position(deg)) < abbs(encoderTarget))
  {
    driveForward(velocity);
    delay(10);
  }
  //shoot();
  //delay(200);
}

void EncoderForward(float forwardValue, float encoderTarget)
{
  LB.resetPosition();
  while (abbs(LB.position(deg)) < abbs(encoderTarget))
  {
    driveForward(forwardValue);
    delay(10);
  }
  driveForward(0);
}

void timerRotate(int v, int t)
{
  float startTime = Brain.timer(msec);
  while (Brain.timer(msec) - t < startTime)
  {
    driveRotate(v);
  }
  driveRotate(0);
}

void EncoderRotate(float forwardValue, float encoderTarget)
{
  LB.resetPosition();
  while (abbs(LB.position(deg)) < abbs(encoderTarget))
  {
    driveRotate(forwardValue);
    delay(10);
  }
  driveRotate(0);
}

float standDev(float a, float b)
{
  return abbs(abbs(a) - abbs(b));
}
float mini(float a, float b)
{
  if (a <= b)
  {
    return a;
  }
  else
  {
    return b;
  }
  // if( c <= a || c <= b || c <= d) return c;
  // if( d <= a || d <= b || d <= c) return d;
}

float getForwardEncoder()
{
  return (abbs(RC.position(deg)) + abbs(LC.position(deg))) / 2.0;
}

void resetLeftEncoder()
{
  LB.resetPosition();
  // ChLM.resetRotation();
  LC.resetPosition();
}
void resetRightEncoder()
{
  RB.resetPosition();
  // ChRM.resetRotation();
  RC.resetPosition();
}
void resetChassisEncoder()
{
  resetLeftEncoder();
  resetRightEncoder();
}

void GyroEncoderForward(float speed, float target, float targetangle)
{
  float errorGyro;
  float errorGyroLimit = 30;
  float kpGyro = 1.8;
  resetChassisEncoder();
  while (abbs(getForwardEncoder()) < target)
  {
    errorGyro = abbs(targetangle - getGyro) > errorGyroLimit ? sign(targetangle - getGyro) * errorGyroLimit : targetangle - getGyro;
    moveForwardRotate(speed, kpGyro * errorGyro);
    delay(10);
  }
  moveForwardRotate(0, 0);
}

void GyroTimerForward(float speed, float time, float targetangle)
{
  float errorGyro;
  float errorGyroLimit = 30;
  float kpGyro = 1.8;
  auto timer = MyTimer();
  resetChassisEncoder();
  while (timer.getTime() < time)
  {
    errorGyro = abbs(targetangle - getGyro) > errorGyroLimit ? sign(targetangle - getGyro) * errorGyroLimit : targetangle - getGyro;
    moveForwardRotate(speed, kpGyro * errorGyro);
    delay(10);
  }
  moveForwardRotate(0, 0);
}

// Gyro Methods
const float kGyro = 1800.0 / 1770.0;
bool resetGyroFlag = false;
void resetGyro() { resetGyroFlag = true; }
float gyroValue = 0;
float getHeading() { return gyroValue; }
float getRoll() { return -Gyro.roll(); }
float gyroBias = 0;
void addGyroBias(float bias) { gyroBias = bias; }
int gyroSensor()
{
  auto myGyro = MyGyro();
  while (true)
  {
    myGyro.update(Gyro.heading());
    if (resetGyroFlag)
    {
      myGyro.reset();
      resetGyroFlag = false;
    }
    if (gyroBias != 0)
    {
      myGyro.addBias(gyroBias);
      gyroBias = 0;
    }
    gyroValue = myGyro.readCalculatedValue() * kGyro;
    // Controller1.Screen.print(gyroValue);
    delay(1);
  }
  return 1;
}

void GyroRotate(int v, int deg)
{
  // resetGyro();
  while (getHeading() < deg)
  {
    driveRotate(v);
    delay(100);
  }
  driveRotate(0);
}

// 赛季特别方法
void timerIndex(float v, int encoderTarget)
{
  itk.resetPosition();
  while (abbs(itk.position(deg)) < abbs(encoderTarget))
  {
    itk.spin(fwd, 120 * v, voltageUnits::mV);
  }
  itk.spin(fwd, 0, voltageUnits::mV);
}

void Rindex(float v, float encoderTarget)
{
  roll1.resetPosition();
  while (abbs(roll1.position(deg)) < abbs(encoderTarget))
  {
    roll1.spin(fwd, 120 * v, voltageUnits::mV);
  }
  roll1.spin(fwd, 0, voltageUnits::mV);
}

void Rroll() // 15s
{
  // opt.setLightPower(100, pct);
  bool color = rollBlue;
  if (color)
  {
    while (!rollRed)
    {
      roll1.spin(fwd, 6000, voltageUnits::mV);
      driveForward(20);
    }
    while (!rollBlue)
    {
      roll1.spin(fwd, 6000, voltageUnits::mV);
      driveForward(20);
    }
  }
  else
  {
    while (!rollBlue)
    {
      roll1.spin(fwd, 6000, voltageUnits::mV);
      driveForward(20);
    }
    while (!rollRed)
    {
      roll1.spin(fwd, 6000, voltageUnits::mV);
      driveForward(20);
    }
  }
  timerIndex(-50, 100);
  driveForward(0);
  // opt.setLightPower(0, pct);
}

void Rroll(bool blue) // 技能赛
{
  auto time = MyTimer();
  if (!blue)
  {
    roll1.spin(fwd, -6000, voltageUnits::mV);
    driveForward(35);
    while (!rollRed && time.getTime() < 1000)
    {
    }
    while (!rollBlue && time.getTime() < 1000)
    {
    }
    timerIndex(50, 200);
    driveForward(0);
  }
  else
  {
    roll1.spin(fwd, -6000, voltageUnits::mV);
    driveForward(35);
    while (!rollBlue && time.getTime() < 1000)
    {
    }
    while (!rollRed && time.getTime() < 1000)
    {
    }
    timerIndex(50, 200);
    driveForward(0);
  }
}
void shoot()
{
  manual = 0;
  autoCata = 1;
  lck = 0;
}

#endif