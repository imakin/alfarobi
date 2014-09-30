#ifndef COMPASSSERIAL_H
#define COMPASSSERIAL_H

#include <SerialStream.h>
#include <sstream>

#define CMPS10_VERSION  7

#define to_string(x) static_cast<std::ostringstream*>( &(std::ostringstream() << x) )->str()

using namespace LibSerial;
using namespace std;

class CompassSerial
{
public:
    static CompassSerial* GetInstance();
    ~CompassSerial();

    int Initialize(const string deviceName);
    int AutoInitialize(int count);

    bool check_version();
    unsigned char get_version();
    unsigned short get_angle();
    unsigned short get_preciseAngle();
    unsigned short get_pitch();
    unsigned short get_roll();

private:
    static CompassSerial* uniqueInstance;
    SerialStream serial;

    unsigned char buf[2];
    char write;
    void get(char &write);
};

#endif // COMPASSSERIAL_H
