#include <iostream>
#include <fstream>
//#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
#include <string.h>
#include <sstream>
#include <cmath>
#include "communication.h"
#include "ObjectPar.h"
#include <cfloat>

using namespace std;

//model parameter
double idle_rpm = 0.0;
double max_rpm = 3200.0;
double max_rpm_change_rate = 1000.0;
double prop_ratio = 0.389105058;
double prop_pitch = 20.0;
double max_rudder_angle = 30.0;
double rudder_coefficient = 0.25;
double rudder_distance = 2.0;
double mass = 2000.0;
double max_power = 8948.4;
double max_speed = 2.75;
//double probability[4] = {0.5,0.3,0.15,0.05};
double probability[4] = {0,1,0,0};
bool debug = true;


struct pointc
{
    double x, y, time;
    pointc(double x1, double y1, double time1)
        : x(x1), y(y1), time(time1){};
};

double max_prop_speed;
double max_force;
double prop_coefficient;
double drag_coefficient;

mutex mtx;
int running = 1;
int plan = 0;
ObjectPar start;
ObjectPar actions[4];
string path = "";
string default_Command = "0,0";
int receivePipeFromParent;
double estimate_effect_speed = 0, estimate_effect_direction = 0;
double ptime = 0;
int iteration = 0;
bool update = true;
vector<pointc> future;

void readpath(FILE *readstream)
{
    int num;
    char locationString[1024];
    fgets(locationString, sizeof locationString, readstream);
    sscanf(locationString, "path %d\n", &num);
    path = locationString;

    while (num--)
    {
        char locationString[1024];
        fgets(locationString, sizeof locationString, readstream);
        path += locationString;
    }
    //for estimate start;
    fgets(locationString, sizeof locationString, readstream);
    path += locationString;
}

void requestAction()
{
    FILE *readstream = fdopen(receivePipeFromParent, "r");
    string s;
    char locationString[1024];

    while (running)
    {
        this_thread::sleep_for(std::chrono::milliseconds(1));
        lock_guard<mutex> lock(mtx);
        fgets(locationString, sizeof locationString, readstream);
        sscanf(locationString, "%lf %lf %lf %lf %lf\n", &start.x, &start.y, &start.heading, &start.speed, &start.otime);

        fgets(locationString, sizeof locationString, readstream);
        sscanf(locationString, "%lf %lf %lf %lf %lf\n", &actions[0].x, &actions[0].y, &actions[0].heading, &actions[0].speed, &actions[0].otime);
        if (actions[0].otime >= 0)
        {
            for (int i = 1; i < 4; i++)
            {
                fgets(locationString, sizeof locationString, readstream);
                sscanf(locationString, "%lf %lf %lf %lf %lf\n", &actions[i].x, &actions[i].y, &actions[i].heading, &actions[i].speed, &actions[i].otime);
            }
        }
        readpath(readstream);

        plan = 1;
    }
}

double radians(double rudder_angle)
{
    return (rudder_angle * M_PI) / 180;
}

double radians_diff(double a, double b)
{
    if (a < 0)
        a = fmod(a + M_PI * 10000, M_PI * 2);
    if (b < 0)
        b = fmod(b + M_PI * 10000, M_PI * 2);
    double diff = a - b;
    diff = fmod((diff + M_PI), M_PI * 2) - M_PI;
    return diff;
}

void estimate(double &rpm, double throttle, double d_time, double &speed, double rudder, double &heading, double &x, double &y)
{
    double target_rpm = idle_rpm + throttle * (max_rpm - idle_rpm);
    double rcr = (target_rpm - rpm) / d_time;

    if (fabs(rcr) > max_rpm_change_rate)
    {
        if (rcr < 0)
            rcr = -max_rpm_change_rate;
        else
            rcr = max_rpm_change_rate;
    }
    rpm += rcr * d_time;

    //seperate this and make a while loop

    double prop_rpm = prop_ratio * rpm;
    double prop_speed = prop_rpm / prop_pitch;
    double rudder_speed = fmax(sqrt(prop_speed), speed);
    double thrust = prop_coefficient * (prop_speed * prop_speed - speed * speed);
    double rudder_angle = rudder * max_rudder_angle;
    double rudder_rads = radians(rudder_angle);
    double thrust_fwd = thrust * cos(rudder_rads);
    double rudder_speed_yaw = rudder_speed * sin(rudder_rads);
    double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

    heading += yaw_rate * d_time;
    heading = fmod(heading, radians(360));
    if (heading < 0)
        heading += radians(360);

    double drag = pow(speed, 3) * drag_coefficient;
    speed += ((thrust_fwd - drag) / mass) * d_time;
    double delta = speed * d_time;
    double deltaEV = estimate_effect_speed * d_time;
    x += delta * sin(heading) + deltaEV * sin(estimate_effect_direction);
    y += delta * cos(heading) + deltaEV * cos(estimate_effect_direction);
}
//take condisder of speed & rmp & heading!!!!! currently do nothing
void MPC(double &r, double &t)
{
    double x = start.x, y = start.y, heading = start.heading, speed = start.speed;
    double rpm = idle_rpm + (speed / max_speed) * (max_rpm - idle_rpm);
    double throttle = 0;
    double rudder = -1;
    double duration = 0.05;

    double coefficient = DBL_MAX;

    if (ptime != start.otime && future.size() != 0 && update)
    {
        if (iteration < 50)
            ++iteration;
        ptime = start.otime;
        double cx = start.x;
        double cy = start.y;
        int index = 0;
        for (int i = 1; i < future.size(); i++)
        {
            if (ptime <= future[i].time)
            {
                index = (fabs(future[i].time - ptime) < fabs(future[i - 1].time - ptime)) ? i : i - 1;
                break;
            }
        }
        double dtime = future[index].time - (future[0].time - 0.05);
        double diffx = (start.x - future[index].x) / dtime;
        double diffy = (start.y - future[index].y) / dtime;
        double deltax = estimate_effect_speed * sin(estimate_effect_direction);
        double deltay = estimate_effect_speed * cos(estimate_effect_direction);
        deltax += diffx / iteration;
        deltay += diffy / iteration;
        estimate_effect_direction = atan2(deltax, deltay);
        double cosd = cos(estimate_effect_direction);
        estimate_effect_speed = (cosd > 0.1) ? deltay / cosd : deltax / sin(estimate_effect_direction);
        if (estimate_effect_direction < 0)
            estimate_effect_direction = fmod(estimate_effect_direction + M_PI * 10000, M_PI * 2);
        else if (estimate_effect_direction > 2 * M_PI)
            estimate_effect_direction = fmod(estimate_effect_direction, M_PI * 2);
    }
    if(debug)
    {
        estimate_effect_speed = 0;
    }
    //  estimate_effect_direction = 1.57;
    //     estimate_effect_speed = 1;
    cerr << "current estimate " << estimate_effect_speed << " " << estimate_effect_direction << endl;
    double choosex, choosey;

    for (int i = -10; i <= 10; ++i)
    {

        rudder = i / 10.0;

        for (int j = 100; j >= 0; --j)
        {
            throttle = j / 100.0;

            double x1 = x, y1 = y, heading1 = heading, speed1 = speed, starttime = 0, rpm1 = rpm, d_time, temp = 0;
            vector<pointc> tempfuture;
            for (int index = 0; index < 4; index++)
            {
                d_time = actions[index].otime - start.otime;
                if(d_time < 0)
                    continue;
                while (starttime + duration < d_time)
                {
                    starttime += duration;
                    estimate(rpm1, throttle, duration, speed1, rudder, heading1, x1, y1);
                    if (tempfuture.size() < 10)
                    {
                        tempfuture.emplace_back(x1, y1, start.otime + starttime);
                    }
                    // cerr << rudder<< " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << endl;
                }
                if (starttime != d_time)
                {
                    estimate(rpm1, throttle, d_time - starttime, speed1, rudder, heading1, x1, y1);
                    //cerr << rudder<< " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << endl;
                }
                temp += (pow(x1 - actions[index].x, 2) + pow(y1 - actions[index].y, 2)) * probability[index];
            }
            if (coefficient > temp)
            {
                r = (int)(rudder * 1000.0) / 1000.0;
                t = (int)(throttle * 1000.0) / 1000.0;
                coefficient = temp;
                future = tempfuture;
            }
            //cerr << temp << " " << rudder << " " << throttle << " " << x1 << " " << y1 << endl;
        }
    }
    //cerr << "action " << action.x << " " << action.y << " target " << choosex << " " << choosey << endl;
    //cerr << "choose coeff " << coefficient << " " << r << " " << t << "\n\n"
      //   << endl;
}

void sendAction()
{
    double rudder, throttle;
    while (running)
    {
        if (getppid() == 1)
        {
            cerr << "CONTROLER TERMINATE" << endl;
            exit(1);
        }
        string command = "";
        if (plan) //use mutex instead of busy waiting
        {
            lock_guard<mutex> lock(mtx);
            if (actions[0].speed == -1)
            {
                command = default_Command;
                update = false;
            }
            else if (actions[0].speed == -2)
            {
                command = "1,0.1";
                update = false;
            }
            else
            {
                MPC(rudder, throttle);
                update = true;
                command = to_string(rudder) + "," + to_string(throttle);
            }
            if (command.size() != 0)
            {
                command = path + "\n" + command;
                cout << command << endl
                     << flush;
            }
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char *argv[])
{
    //prepare fore model compute
    max_prop_speed = (max_rpm * prop_ratio) / prop_pitch;
    max_force = max_power / max_speed;
    prop_coefficient = max_force / (max_prop_speed * max_prop_speed - max_speed * max_speed);
    drag_coefficient = max_force / (pow(max_speed, 3));

    receivePipeFromParent = stoi(argv[1]);
    if (argc > 3)
    {
        ifstream f(argv[3]);
        if (f.is_open())
        {
            f >> max_rpm >> max_power >> idle_rpm >> prop_ratio >> prop_pitch >> max_rpm_change_rate >> max_speed >> mass >> max_rudder_angle >> rudder_distance >> rudder_coefficient;
        }
    }
    cout.precision(numeric_limits<float>::digits10 + 2);
    cerr.precision(numeric_limits<float>::digits10 + 2);
    thread thread_for_executive(thread([=] { requestAction(); }));

    sendAction();
    thread_for_executive.join();
    return 0;
}
