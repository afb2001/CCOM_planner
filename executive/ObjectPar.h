#ifndef OBJECTPAR_H
#define OBJECTPAR_H
#include <string>
#include <iostream>


struct point{
    int x, y;
    point()
    :x(0),y(0) {}

    point(int x,int y)
    :x(x),y(y) {}

    bool operator==(const point &s) const
    {
        if (x == s.x && y == s.y)
            return true;
        return false;
    }
};

namespace std
{

template <>
struct hash<point>
{
    hash()
    {
    }
    size_t operator()(const point &c) const
    {
        int x = c.x, y = c.y;
        unsigned long value = 0;
        for (int i = 0; i < 4; i++)
        {
            if (!x)
                break;
            value += value * 31 + (x & 8);
            x = x >> 8;
        }
        for (int i = 0; i < 4; i++)
        {
            if (!y)
                break;
            value += value * 31 + (y & 8);
            y = y >> 8;
        }
        return value;
    }
};
}

class ObjectPar
{

  public:
    double x, y, heading, speed, otime;
    ObjectPar(double x, double y, double heading, double speed, double otime)
        : x(x), y(y), heading(heading), speed(speed), otime(otime){};
    ObjectPar()
        : x(-1), y(-1), heading(-1), speed(-1), otime(-1){};

    void set(double newx, double newy, double newheading, double newspeed, double newtime)
    {
        x = newx;
        y = newy;
        heading = newheading;
        speed = newspeed;
        otime = newtime;
    }

    std::string toString()
    {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(heading) + " " + std::to_string(speed) + " " + std::to_string(otime);
    }

    void print()
    {
        std::cout << x << " " << y << " " << heading << " " << speed << " " << otime << std::endl;
    }

    void printerror()
    {
        std::cerr << x << " " << y << " " << heading << " " << speed << " " << otime << std::endl;
    }
};

#endif