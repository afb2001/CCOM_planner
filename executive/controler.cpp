#include <iostream>
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
#define idle_rpm 0.0
#define max_rpm 3200.0
#define max_rpm_change_rate 1000.0
#define prop_ratio 0.389105058
#define prop_pitch 20.0
#define max_rudder_angle 30.0
#define rudder_coefficient 0.25
#define rudder_distance 2.0
#define mass 2000.0
#define max_power 8948.4
#define max_speed 2.75

double max_prop_speed;
double max_force;
double prop_coefficient;
double drag_coefficient;

mutex mtx;
int running = 1;
int send = 0;
int plan = 0;
ObjectPar start;
ObjectPar action;
ObjectPar oldaction = ObjectPar(0, 0, 0, 0, 0);
vector<ObjectPar> dyamic_obstacles;
string path = "";
double currentTimestamp = -1, previous_location_time = -1;
double currenttime_for_action = -1;
string pheading = "0";
string previousrequestString;
string default_Command = "0,0";
int sendPipeToParent, receivePipeFromParent;

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

    while (running)
    {
        char locationString[1024];
        mtx.lock();

        fgets(locationString, sizeof locationString, readstream);
        sscanf(locationString, "%lf %lf %lf %lf %lf\n", &start.x, &start.y, &start.heading, &start.speed, &start.otime);
        //cerr << "CONTROLLER::Start" << locationString << endl;
        fgets(locationString, sizeof locationString, readstream);
        sscanf(locationString, "%lf %lf %lf %lf %lf\n", &action.x, &action.y, &action.heading, &action.speed, &action.otime);
        readpath(readstream);

        // start.printerror();
        // cerr << "CONTROLLER :: action ";
        // action.printerror();
        plan = 1;
        mtx.unlock();
    }
}

double radians(double rudder_angle)
{
    return (rudder_angle * M_PI) / 180;
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
    //thrust = random.gauss(thrust,thrust*0.1);
    double rudder_angle = rudder * max_rudder_angle;
    //rudder_angle += random.gauss(0.0,0.25)
    double rudder_rads = radians(rudder_angle);
    double thrust_fwd = thrust * cos(rudder_rads);
    // cerr << thrust_fwd  << "  " << thrust << endl;
    double rudder_speed_yaw = rudder_speed * sin(rudder_rads);
    double yaw_rate = rudder_coefficient * rudder_speed_yaw / rudder_distance;

    heading += yaw_rate * d_time;
    heading = fmod(heading, radians(360));
    if (heading < 0)
        heading += radians(360);

    double drag = pow(speed, 3) * drag_coefficient;
    //drag = random.gauss(drag,drag*0.1)
    //a = (thrust_fwd-drag)/mass;
    speed += ((thrust_fwd - drag) / mass) * d_time;
    // if (speed > 0)
    //     (prop_rpm/prop_pitch)/speed;
    double delta = speed * d_time;
    x = x + delta * sin(heading);
    y = y + delta * cos(heading);
}
//take condisder of speed & rmp & heading!!!!! currently do nothing
void MPC(double &r, double &t)
{
    double x = start.x, y = start.y, heading = start.heading, speed = start.speed;
    double rpm = idle_rpm + (speed / max_speed) * (max_rpm - idle_rpm);

    double throttle = 0;
    double rudder = -1;
    double duration = 0.05;
    double d_time = action.otime - start.otime;
    double coefficient = DBL_MAX;
    // cerr << d_time << endl;

    //cerr << "START " << endl;

    for (rudder = -1; rudder <= 1; rudder += 0.01)
    {
        for (throttle = 1; throttle >= 1; throttle -= 0.01)
        {
            double x1 = x, y1 = y, heading1 = heading, speed1 = speed, starttime = 0, rpm1 = rpm;
            while (starttime + duration < d_time)
            {
                starttime += duration;
                estimate(rpm1, throttle, duration, speed1, rudder, heading1, x1, y1);
                // cerr << rudder<< " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << endl;
            }
            if (starttime != d_time)
            {
                estimate(rpm1, throttle, d_time - starttime, speed1, rudder, heading1, x1, y1);
                // cerr << rudder<< " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << endl;
            }
            double temp = sqrt(pow(x1 - action.x, 2) + pow(y1 - action.y, 2));
            if (coefficient > temp)
            {
                r = (int)(rudder * 1000.0) / 1000.0;
                t = (int)(throttle * 1000.0) / 1000.0;
                coefficient = temp;
            }
            //cerr << rudder << " " << throttle << " " << x1 << " " << y1 << " " << speed1 << " " << heading1 << " " << temp << endl;
        }
    }

    // cerr << "TARGET " << endl;
    // cerr << r << " " << t << " " << coefficient << endl;
    // cerr << start.x << " " << start.y << " " << start.speed << " " << start.heading << endl;
    // cerr << action.x << " " << action.y << " " << action.speed << " " << action.heading << endl;
    // cerr << "END " << endl;
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

            mtx.lock();
            //cerr << action.x << " " << action.y << " " << action.speed << " " << action.heading << endl;
            if (action.speed == -1)
            {
                command = "0,0";
            }
            else
            {
                MPC(rudder, throttle);
                //command = "0,0";
                command = to_string(rudder) + "," + to_string(throttle);
            }

            if (command.size() != 0)
            {
                command = path + "\n" + command;
                cout << command << endl
                     << flush;

                //cerr << command << endl;
            }
            mtx.unlock();
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
    sendPipeToParent = stoi(argv[2]);
    cout.precision(numeric_limits<float>::digits10 + 2);
    cerr.precision(numeric_limits<float>::digits10 + 2);
    //cerr << "CONTROLLER INITIALIZE" << endl;
    thread thread_for_executive(thread([=] { requestAction(); }));

    sendAction();
    thread_for_executive.join();
    //thread thread_for_UDVSEND(thread([=] { sendAction(); }));
    //thread thread_for_UDVLOC(thread([=] { requestLocation(); }));
    //this_thread::sleep_for(std::chrono::milliseconds(1000));
    //thread_for_UDVSEND.join();
    //thread_for_UDVLOC.join();
    return 0;
}
