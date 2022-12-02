#ifndef TASKS_H
#define TASKS_H

//#include "GPS_functions.h"
#include "definitions_and_declarations.h"
// using namespace Eigen;

namespace vex
{
    class card : brain::sdcard
    {
    public:
        double nwritten = 0;
        double nread = 0;
        uint8_t Data[1];
        uint8_t Readbuffer[1000];
        int read(int *a);
        int init(int a);
        int append(int a);
        void reset(void);
    };
}

int card::read(int *a)
{

    this->nread = Brain.SDcard.loadfile("t.h", this->Readbuffer, sizeof(this->Readbuffer));
    int i = 0;
    // *a = this->Readbuffer[0];
    while (this->Readbuffer[i] != 0)
    {
        *a = this->Readbuffer[i];
        // Brain.Screen.printAt(10, 60, "readBuffer: %d", this->Readbuffer[i]);
        i++;
        // delay(2000);
    }
    return this->nread;
}

int card::init(int a)
{
    this->Data[0] = a;
    this->nwritten = Brain.SDcard.savefile("t.h", this->Data, sizeof(this->Data));
    return this->nwritten;
}

int card::append(int a)
{
    this->Data[0] = a;
    this->nwritten = Brain.SDcard.appendfile("t.h", this->Data, sizeof(this->Data));
    return this->nwritten;
}

void card::reset()
{
    memset(this->Data, '\0', sizeof(this->Data));
    memset(this->Readbuffer, '\0', sizeof(this->Readbuffer));
    this->nwritten = 0;
    this->nread = 0;
}

void Ch()
{
    switch (ch_state)
    {
    case 0:
        LB.spin(fwd, 120 * (Ch1 * rota_0 + Ch3), voltageUnits::mV);
        LC.spin(fwd, 120 * (Ch1 * rota_0 + Ch3), voltageUnits::mV);
        RB.spin(fwd, 120 * (Ch1 * rota_0 - Ch3), voltageUnits::mV);
        RC.spin(fwd, 120 * (Ch1 * rota_0 - Ch3), voltageUnits::mV);
        break;
    case 1:
        LB.spin(fwd, 120 * (Ch1 * rota_1 + Ch3), voltageUnits::mV);
        LC.spin(fwd, 120 * (Ch1 * rota_1 + Ch3), voltageUnits::mV);
        RB.spin(fwd, 120 * (Ch1 * rota_1 - Ch3), voltageUnits::mV);
        RC.spin(fwd, 120 * (Ch1 * rota_1 - Ch3), voltageUnits::mV);
        break;
    case 2:
        LB.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
        LC.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
        RB.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
        RC.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
        break;
    case 3:
        // LA.spin(fwd, 120*(Ch1+Ch3), voltageUnits::mV);
        LB.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
        LC.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
        // RA.spin(fwd, 120*(Ch1-Ch3), voltageUnits::mV);
        RB.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
        RC.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
        break;
    case 4:
       
    break;
    }
}

void intake(float speed)
{
    itk.spin(fwd, -120 * speed, voltageUnits::mV);
}

void cata(float speed)
{
    cat.spin(fwd, 120 * speed, voltageUnits::mV);
    cat2.spin(fwd, -120 * speed, voltageUnits::mV);
}

// void index(float speed){
//     ind.spin(fwd, 120*speed, voltageUnits::mV);
// }

int AutoCata()
{
    float startTime;
    int t = 2500;
    while (1)
    {
        if (autoCata)
        {
            lck = 0;
            manual = 0;
            startTime = Brain.timer(msec);
            while ((getDis <= targetDis) && (Brain.timer(msec) - 1000 < startTime))
            {
                cata(-100);
                delay(10);
            }
            delay(100);
            while ((getDis >= targetDis) && (Brain.timer(msec) - t < startTime) && !AutoCataInterrupt)
            {

                float velocity = abbs(getDis - targetDis) > 50 ? -80 : fmin(-50, -2.1 * (getDis - targetDis));
                cata(velocity);
                delay(10);
            }
            cata(0);
            lckReset = 1;
            lck = 1;
            autoCata = getDis > targetDis ? 1 : 0;
            manual = 1;
            AutoCataInterrupt = 0;
        }
        delay(10);
    }
}

int LCK()
{
    lck = 1;
    lckReset = 1;
    float target = 0;
    float kp = 2; // 2
    float pow;
    while (1)
    {
        while (lck)
        {
            if (lckReset)
            {
                target = getCataEncoder;
                lckReset = 0;
            }
            pow = kp * (target - getCataEncoder);
            cata(pow);
            delay(25);
        }
        delay(10);
    }
}
int adjustDis()
{
    /*
    --------------------------------------WARNING--------------------------------------------
    You need to delete the flie in SDcard named "t.h", if you have used function WRITE for
    many times.(approximately 1000 times) Thanks to the VEX V5 bugs with SDcard, you have to
    do so if you want to play with SDcard; You can change file name by changing definition
    "SDFile" in "definition_and_declarations.h" if you don't like "t.h". By the way, Jpearman
    said some SDcard can be read by V5 brain while some can not and it dont know why, so test
    & try the right SDcard. The SDcard should be formatted in FAT32.
    -----------------------------------------------------------------------------------------
    */
    bool lastUP = 0, lastDOWN = 0, lastRIGHT = 0;
    int diss;
    card card1;
    float initDis = 32;
    int resetDis = 60;
    card1.reset();
    /*
    -----------------------------------Init SD card------------------------------------------
    Reading SDcard when start running program if card inserted, otherwise init targetDis 32,
    change variable "initDis" if you want to adjust init value.
    -----------------------------------------------------------------------------------------
    */
    if (isFileExists && isSDInserted)
    {
        card1.reset();
        card1.read(&diss);
        Brain.Screen.printAt(10, 40, "sdDATA: %d", diss);
        targetDis = float(diss);
    }
    else
    {
        targetDis = initDis;
    }
    /*
    -------------------------------------Functions-------------------------------------------
    Hold LEFT && press UP       : targetDis += 2(mm) && arm reachs updated targetDis.
    Hold LEFT && press DOWN     : targetDIs -= 2(mm) && arm reachs updated targetDis.
    Hold LEFT && press RIGHT    : WRITE targetDis into SD card && print if successful on Brain
                                  screen, if so it will print the data in SDcard.
    Hold DOWN && press UP       : if SDcard inserted, reset SDcard to resetDis(60 default) &&
                                  update targetDis;
                                  if SDcard not inserted, reset targetDis to resetDis.
    -----------------------------------------------------------------------------------------
    */
    while (1)
    {
        if (UP && DOWN && lastUP)
        {
            if (isSDInserted)
            {
                card1.reset();
                if (isFileExists)
                {
                    card1.append(resetDis);
                }
                else
                {
                    card1.init(resetDis);
                }
                card1.reset();
                card1.read(&diss);
                targetDis = float(diss);
                Brain.Screen.printAt(10, 40, "sdDATA: %d", diss);
            }
            else
            {
                targetDis = resetDis;
            }
        }
        if (LEFT && ((UP && !lastUP) || (DOWN && !lastDOWN)))
        {
            if (UP)
            {
                targetDis += 2;
            }
            if (DOWN)
            {
                targetDis -= 1;
            }
            if (targetDis > 10)
            {
                manual = 0;
                lck = 0;
                cata(100);
                delay(100);
                autoCata = 1;
            }
        }
        if (LEFT && (RIGHT && !lastRIGHT))
        {
            if (isSDInserted)
            {
                card1.reset();
                if (isFileExists)
                {
                    card1.append(targetDis);
                }
                else
                {
                    card1.init(targetDis);
                }
                Brain.Screen.printAt(10, 20, "Write successful! ");
                card1.reset();
                card1.read(&diss);
                Brain.Screen.printAt(10, 40, "sdDATA: %d", diss);
                targetDis = float(diss);
            }
            else
            {
                Brain.Screen.printAt(10, 20, "No SDcard inserted");
            }
        }
        lastUP = UP;
        lastDOWN = DOWN;
        lastRIGHT = RIGHT;
        delay(10);
    }
}

// int test(){
//     card card1;

// }

#endif