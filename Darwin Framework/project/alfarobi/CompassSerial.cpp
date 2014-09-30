#include "CompassSerial.h"

CompassSerial* CompassSerial::uniqueInstance = new CompassSerial();

CompassSerial *CompassSerial::GetInstance()
{
    return uniqueInstance;
}

int CompassSerial::Initialize(const string deviceName)
{
    serial.Open(deviceName);
    if (serial.IsOpen())
    {
        serial.SetBaudRate(SerialStreamBuf::BAUD_9600);
        serial.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
        serial.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
        serial.SetNumOfStopBits(2);
        serial.SetParity(SerialStreamBuf::PARITY_NONE);
    } else
    {
        fprintf(stderr, "can't open serial\n");
        return -1;
    }
    buf[0] = 0;
    return 0;
}

int CompassSerial::AutoInitialize(int count)
{
    static string USB;

    for (int var = 0; var < count;) {
        USB = "/dev/ttyUSB";
        USB += to_string(var);
        printf("count : %d", count);
        if(Initialize(USB) == 0 && check_version()) return var;
        serial.Close();
        var++;
    }
    return -1;
}

//~ int CompassSerial::AutoInitialize(int count)
//~ {
    //~ static string USB;
//~ 
    //~ for (int var = 0; var < count;) {
        //~ USB = "/dev/ttyUSB";
        //~ USB += to_string(var);
        //~ if(Initialize(USB) == 0 && check_version()) return 0;
        //~ serial.Close();
        //~ var++;
    //~ }
    //~ return -1;
//~ }

bool CompassSerial::check_version()
{
    serial.SetVMin(0);
    serial.SetVTime(5);
    return (get_version() == CMPS10_VERSION ? true : false);
}

unsigned char CompassSerial::get_version()
{
    write = 0x11;
    get(write);
    return buf[0];
}

unsigned short CompassSerial::get_angle()
{
    write = 0x12;
    get(write);
    return buf[0]*360/255;
}

// BUG : get_preciseAngle() can't read properly and make other fucntion mesh
unsigned short CompassSerial::get_preciseAngle()
{
    buf[0] = NULL;
    buf[1] = NULL;
    write = 0x13;
    serial.write(&write, 1);
    serial.read((char*)&buf, 2);
    return ((buf[1]<<8) + buf[0]);
}

unsigned short CompassSerial::get_pitch()
{
    write = 0x14;
    get(write);
    return buf[0]*360/255;
}

unsigned short CompassSerial::get_roll()
{
    write = 0x15;
    get(write);
    return buf[0]*360/255;
}

void CompassSerial::get(char &write)
{
    serial.write(&write, 1);
    serial.read((char*)&buf[0], 1);
}


/*
#include "CompassSerial.h"

CompassSerial* CompassSerial::uniqueInstance = new CompassSerial();

CompassSerial *CompassSerial::GetInstance()
{
    return uniqueInstance;
}

int CompassSerial::Initialize(const string deviceName)
{
    serial.Open(deviceName);
    if (serial.IsOpen())
    {
        serial.SetBaudRate(SerialStreamBuf::BAUD_9600);
        serial.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
        serial.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
        serial.SetNumOfStopBits(2);
        serial.SetParity(SerialStreamBuf::PARITY_NONE);
    } else
    {
        fprintf(stderr, "can't open serial\n");
        return -1;
    }
    buf[0] = 0;
    return 0;
}

int CompassSerial::AutoInitialize(int count)
{
    static string USB;

    for (int var = 0; var < count;) {
        USB = "/dev/ttyUSB";
        USB += to_string(var);
        //~ printf("count : %d", count);
        if(Initialize(USB) == 0 && check_version()) return var;
        serial.Close();
        var++;
    }
    return -1;
}

bool CompassSerial::check_version()
{
    serial.SetVMin(0);
    serial.SetVTime(5);
    return (get_version() == CMPS10_VERSION ? true : false);
}

unsigned char CompassSerial::get_version()
{
    write = 0x11;
    if(get(write)) return buf[0];
    else return 404;
}

unsigned short CompassSerial::get_angle()
{
    write = 0x12;
    if(get(write)) return buf[0]*360/255;
    else 404;
}

// BUG : get_preciseAngle() can't read properly and make other fucntion mesh
unsigned short CompassSerial::get_preciseAngle()
{
    buf[0] = NULL;
    buf[1] = NULL;
    write = 0x13;
    serial.write(&write, 1);
    serial.read((char*)&buf, 2);
    return ((buf[1]<<8) + buf[0]);
}

unsigned short CompassSerial::get_pitch()
{
    write = 0x14;
    if(get(write)) return buf[0]*360/255;
    else return 404;
}

unsigned short CompassSerial::get_roll()
{
    write = 0x15;
    if(get(write)) return buf[0]*360/255;
    else return 404;
}

bool CompassSerial::get(char &write)
{
    if(serial.IsOpen()) {
        serial.write(&write, 1);
        serial.read((char*)&buf[0], 1);
        return true;
    } else
        return false;
}
*/
