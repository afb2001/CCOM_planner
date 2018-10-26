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

using namespace std;

mutex mtx;
int running = 1;
int send = 0;
int plan = 0;
ObjectPar start;
ObjectPar action;
ObjectPar  oldaction= ObjectPar(0,0,0,0,0);
vector<ObjectPar> dyamic_obstacles;
string path = "";
double currentTimestamp = -1,previous_location_time = -1;
double currenttime_for_action = -1;
string pheading = "0";
string previousrequestString;
string default_Command = "0,0";
int sendPipeToParent, receivePipeFromParent;

void readpath(FILE* readstream)
{
    int num;
    char locationString[1024];
    fgets(locationString, sizeof locationString, readstream);
    sscanf(locationString, "path %d\n", &num);
    path = locationString;
    
    while(num--)
    {
        char locationString[1024];
        fgets(locationString, sizeof locationString, readstream);
        path+=locationString;
    }
    //for estimate start;
    fgets(locationString, sizeof locationString, readstream);
    path+=locationString;
    
}

void requestAction()
{
    FILE *readstream = fdopen (receivePipeFromParent, "r");
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
        //cerr << "CONTROLLER::action" << locationString << endl;
        readpath(readstream);
        
        // start.printerror();
        // cerr << "CONTROLLER :: action ";
        // action.printerror();
        plan = 1;
        mtx.unlock();
    }
}

// void requestLocation()
// {
//     int lengh = 256, h;
//     char locationString[256];
    
//     while (running)
//     {
//         sscanf(locationString, "rx: %lf,%lf,%lf,%lf,%lf [%d]", &start.x, &start.y, &start.heading, &start.speed, &start.otime, &h);
//         oldaction = start;
//         cerr << "START " << endl;
//         //start.print();
//     }
// }

void sendAction()
{
    //Communitcation communication_With_Controler("controler", 1, 1, 1);
    
    while (running)
    {   
        if(getppid() == 1)
        {
            cerr << "CONTROLER TERMINATE" << endl;
            exit(1);
        } 
        string command = "";
        if (plan)//use mutex instead of busy waiting
        {
            mtx.lock();
            if (fabs(currentTimestamp - action.otime) < 0.0000001 && fabs(previous_location_time - start.otime) < 0.0000001)
            {
                
                if (currenttime_for_action > action.otime)
                {
                    command += pheading + ",0";
                }
                else
                {
                    command += previousrequestString;
                    currenttime_for_action += 0.05;
                }
            }
            else if(start.otime > action.otime)
            {
                command += pheading + ",0";
            }
            else
            {
                currentTimestamp = action.otime;
                previous_location_time = start.otime;
                // if(oldaction.otime < start.otime)
                // {
                //     oldaction = start;
                // }
                oldaction = start;
                double ydis = action.y - start.y;
                double xdis = action.x - start.x;
                // cerr << "CONTROLER " << start.x << " " << start.y  << " "<< action.x << " " << action.y << endl;
                float heading = atan2(xdis, ydis);
                //cerr << "CONTROLER " <<ydis << " "<< xdis << " " << heading << endl;
                if(heading < 0)
                {
                    heading += M_PI*2;
                }
                double speed = sqrt(xdis * xdis + ydis * ydis) / (fabs(action.otime - oldaction.otime));

                currenttime_for_action = oldaction.otime + 0.05;
                //oldaction = action;
                //mutex for start and action
                pheading = to_string(heading);
                if(action.speed == 0)
                {
                    speed = 0;
                    pheading = to_string(action.heading);
                }
                    
                previousrequestString = pheading + "," + to_string(speed);
                // pheading = to_string(action.heading);
                // previousrequestString = pheading + "," + to_string(action.speed);
                command += previousrequestString;
                //cerr << "CONTROLER:: " <<command << endl;
            //    cerr << "CONTROLLER Command " << command <<  " " << speed << " " << heading << endl;
            }
            if(command.size() != 0)
            {
                command = path + "\n"+ command;
                cout << command << endl << flush;
               
                //cerr << command << endl;
            }
            mtx.unlock();
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main(int argc, char *argv[])
{   
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
