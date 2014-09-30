#ifndef ALFAROBICOMMUNICATION_H
#define ALFAROBICOMMUNICATION_H

//#include "RoboCupGameControlData.h"

//==================================================
/*
 *  DATA KOMUNIKASI ANTAR TEMAN
 *  NOTE : ...
 */
typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

namespace Alfarobi {

#define ALFAROBI_STRUCT_HEADER  "AGme"
#define AlfaStruct2String(x) ((unsigned char*)&x)

struct EnemyGoal
{
    uint8 distance;
    uint8 angle;
};

struct Ball
{
    uint16 distance;
    uint8 angle;
//    EnemyGoal goal;
};

struct Compass
{
    short pitch;
    short roll;
    short yaw;
};

struct Kiper
{
	uint8 posisi_bola;      // (posisi bola di quadran ke berapa)
};

struct Player
{
    char header[4];
    uint16 arah;            // bearing degres pembacaan compass
    bool flag_bolaDekat;    // data dari/ke assist (bek or striker)
    Kiper alfa;
    //~ Compass kompas;
//    EnemyGoal goal;     //
//    Ball ball;

    // ----==== FUTURE : kedepannya kalau udah belajar koordinat mapping algorithm
    //    uint8 position_x;
    //    uint8 position_y;
};

}   


#endif // ALFAROBICOMMUNICATION_H
