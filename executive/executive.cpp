#include <iostream>
//#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
#include <sstream>
#include <cmath>
#include <ctime>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include "ObjectPar.h"
#include "communication.h"
#include <fstream>
#include <list>

using namespace std;

mutex mtx_path;
mutex mtx_obs;
mutex mtx_cover;

int running = 1;
int pathindex = 0;
int request_start = 0;
int request_start1 = 0;
int countn = 0;
int send = 1;
bool newPath = 0;
double difx = 0, dify = 0;
double previousheading = 0;

ObjectPar pstart;
ObjectPar previousAction;
ObjectPar estimateStart;
ObjectPar current_location;
ObjectPar action;

vector<ObjectPar> dyamic_obstacles;
vector<ObjectPar> path;
vector<ObjectPar> newpath;

list<point> cover;
list<point> newcover;

Communitcation communication_With_Planner, communication_With_Controler;

double getCurrentTime()
{
    //change to clock_gettime
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

void checkTerminate()
{
    if (getppid() == 1)
    {
        cerr << "excutive TERMINATE" << endl;
        running = 0;
        exit(1);
    }
}

void findStart()
{
    mtx_path.lock();
    int path_size = path.size();
    bool find = false, visit = true;
    if (path_size > 0 && pathindex < path_size)
    {
        ObjectPar current = current_location;
        double otime = current_location.otime + 4;

        for (int i = pathindex; i < path_size; i++)
        {

            if (path[i].otime > otime - 3 && i != 0 && visit)
            {
                estimateStart = ObjectPar(path[i].x, path[i].y, path[i].heading, path[i].speed, otime - 3);
                visit = false;
            }
            else if (path[i].otime > otime && i != 0)
            {
                double timeiterval = 1; //otime - path[i - 1].otime;
                double predictHead = path[i].heading;
                while (predictHead < 0)
                    predictHead += 2 * M_PI;
                predictHead = fmod(predictHead, 2 * M_PI);
                action = ObjectPar(path[i].x, path[i].y, predictHead, path[i].speed, path[i].otime); //chage to relative heading
                find = true;
                break;
            }
        }
    }

    if (!find)
    {
        //cerr << "SECOND" << endl;
        ObjectPar current = current_location;
        double timeiterval = 1;
        //double heading = atan2(previousAction.x - pstart.x, previousAction.y - pstart.y);
        //estimateStart = ObjectPar(current.x + timeiterval * previousAction.speed * cos(heading), current.y + timeiterval * previousAction.speed * sin(heading), current.heading, current.speed, current.otime + 1);
        if (visit)
            estimateStart = ObjectPar(current.x + timeiterval * cos(current.heading) * current.speed, current.y + timeiterval * sin(current.heading) * current.speed, current.heading, current.speed, current.otime + 1);
        action = ObjectPar(-1, -1, -1, -1, -1);
        // if (path.size() > pathindex && path[pathindex].speed == 0)
        //     action = ObjectPar(-1, -1, -1, -1, -1);
        // else
        //     action = ObjectPar(current.x + timeiterval * previousAction.speed * cos(heading), current.y + timeiterval * previousAction.speed * sin(heading), current.heading, current.speed, current.otime + 1);
    }

    mtx_path.unlock();
}

// fix the moving of start
void requestPath()
{
    string s;
    int numberOfState;
    double x, y, heading, speed, otime;
    FILE *readstream = fdopen(communication_With_Planner.getWpipe(), "r");
    char response[1024];
    while (!request_start || !request_start1)
        this_thread::sleep_for(std::chrono::milliseconds(50));

    previousAction = current_location;
    pstart = current_location;
    action = current_location;
    estimateStart = current_location;
    estimateStart.otime += 1;
    while (running)
    {

        vector<ObjectPar> newpath;
        communication_With_Planner.cwrite("plan");
        mtx_cover.lock();
        int size = newcover.size();
        communication_With_Planner.cwrite("newly covered " + to_string(size));
        for (point p : newcover)
        {
            communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));
            cerr << "EXECUTIVE::NEWLYCOVERD:: " << p.x << " " << p.y << endl;
        }
        newcover.clear();
        mtx_cover.unlock();
        double start = getCurrentTime();
        findStart();
        //cerr << "REAL::" << current_location.toString() << endl;
        //cerr << "EXECUTIVE::SENDSTART::" << estimateStart.toString() << endl;
        communication_With_Planner.cwrite("start state " + estimateStart.toString());
        mtx_obs.lock();
        int d_obstacles_size = dyamic_obstacles.size();
        s = "dynamic obs " + to_string(d_obstacles_size);
        communication_With_Planner.cwrite(s);

        for (int i = 0; i < d_obstacles_size; i++)
        {
            s = to_string(i);
            s += " " + dyamic_obstacles[i].toString();
            communication_With_Planner.cwrite(s);
        }

        mtx_obs.unlock();

        fgets(response, sizeof response, readstream);

        if (!strncmp(response, "done", 4))
        {
            running = 0;
            break;
        }

        sscanf(response, "plan %d\n", &numberOfState);
        if (numberOfState == 0)
        {
            this_thread::sleep_for(chrono::milliseconds(50));
            checkTerminate();
            continue;
        }
        mtx_path.lock();

        double time_bound = current_location.otime;
        ObjectPar current = current_location;
        int path_size = path.size();
        bool unvisit = true;
        double sleeptime = 0;
        ObjectPar p;
        double diffx = 0, diffy = 0;

        for (int i = pathindex; i < path_size; i++)
        {
            if (path[i].otime >= time_bound && path[i].otime <= estimateStart.otime)
                newpath.push_back(path[i]);
            else
                break;
        }
        for (int i = 0; i < numberOfState; i++) // if no new path then keep old path
        {
            fgets(response, sizeof response, readstream);
            sscanf(response, "%lf %lf %lf %lf %lf\n", &x, &y, &heading, &speed, &otime);
            if (time_bound > otime)
                continue;

            // cerr << ObjectPar(x,y,heading,speed,otime).toString()<< endl;

            if (unvisit)
            {
                int index = newpath.size();
                if (index != 0)
                {
                    sleeptime = estimateStart.otime - time_bound; // newpath[index - 1].otime;
                    if (sleeptime < 0)
                        sleeptime = 0;
                }
                ObjectPar p;
                int j = -1;
                bool find = false;
                for (j = pathindex; j < path_size; j++)
                {
                    if (path[j].otime > estimateStart.otime)
                    {
                        p = path[j];
                        find = true;
                        break;
                    }
                }
                if (!find)
                {
                    diffx = 0;
                    diffy = 0;
                }
                else
                {
                    double timeiterval = current.otime - estimateStart.otime;
                    double angle = atan2(current.y - estimateStart.y, current.x - estimateStart.x);
                    diffx = current.x + timeiterval * current.speed * cos(angle) - estimateStart.x;
                    diffy = current.y + timeiterval * current.speed * sin(angle) - estimateStart.y;

                    //heading

                    //cerr << "DIFFX " << diffx << " " << current.x << " " << estimateStart.x << endl;
                    //cerr << "DIFFY " << diffy << " " << current.y << " " << estimateStart.y << endl;
                }
                unvisit = false;
            }
            p = ObjectPar(x + diffx, y + diffy, heading, speed, otime);
            newpath.push_back(p);
        }
        // cerr << "END" << endl;
        if (!unvisit)
        {
            path = newpath;
            pathindex = 0;
        }
        mtx_path.unlock();
        double end = getCurrentTime();
        if (end - start <= 1)
            this_thread::sleep_for(chrono::milliseconds(((int)((1 - (end - start)) * 1000))));
        checkTerminate();
    }
}

void requestWorldInformation()
{
    char locationString[8192];
    double x, y, heading, speed, otime;
    int index, h, oldbytesRead, bytesRead;
    while (running) // should clear the previous one prevent the disappear obs
    {
        read(STDIN_FILENO, locationString, 8192);

        if (!strncmp(locationString, "Location", 8))
        {
            request_start = 1;
            sscanf(locationString + 9, "%lf,%lf,%lf,%lf,%lf [%d]", &current_location.x, &current_location.y, &current_location.speed, &current_location.heading, &current_location.otime, &h);
            mtx_cover.lock();
            auto it = cover.begin();
            while (it != cover.end())
            {
                float x = it->x - current_location.x;
                float y = it->y - current_location.y;
                if (x * x + y * y <= 10)
                {
                    auto it1 = it;
                    newcover.push_back(*it);
                    ++it;
                    cover.erase(it1);
                }
                else
                    ++it;
            }
            mtx_cover.unlock();
        }
        else if (!strncmp(locationString, "Obstacle", 8))
        {
            request_start1 = 1;
            bytesRead = 9;
            oldbytesRead = bytesRead;
            mtx_obs.lock();
            int d_obstacles_size = dyamic_obstacles.size();
            while (sscanf(locationString + bytesRead, "%d,%lf,%lf,%lf,%lf,%lf\n%n", &index, &x, &y, &speed, &heading, &otime, &bytesRead) == 6)
            {
                bytesRead += oldbytesRead;
                oldbytesRead = bytesRead;
                index < d_obstacles_size ? dyamic_obstacles[index].set(x, y, heading, speed, otime) : dyamic_obstacles.push_back(ObjectPar(x, y, heading, speed, otime));
            }
            d_obstacles_size = dyamic_obstacles.size();
            while (index >= d_obstacles_size)
                dyamic_obstacles.pop_back();
            mtx_obs.unlock();
        }
        else
        {
            cerr << "EXECUTIVE::ERROR REQUEST" << endl;
            cerr << locationString << endl;
            cerr << "END ERROR" << endl;
        }
        checkTerminate();
    }
}

void sendPath(string &s)
{
    int size = path.size() - pathindex;
    int size1 = path.size();
    s += "path " + to_string(size) + "\n";
    for (int i = pathindex; i < size1; i++)
    {
        s += path[i].toString();
        if (size1 - 1 != i)
            s += '\n';
    }
    s += "\n" + estimateStart.toString(); //for estimate start
    s += '\0';
}

void sendAction()
{
    while (running)
    {

        mtx_path.lock();
        int path_size = path.size();

        if (send && action.otime > current_location.otime)
        {
            string s = "";
            pstart = current_location;
            s += pstart.toString() + "\n";
            while (path_size > pathindex && pstart.otime > path[pathindex].otime)
                pathindex++;
            if (path_size == pathindex)
            {
                mtx_path.unlock();
                this_thread::sleep_for(std::chrono::milliseconds(50));
                checkTerminate();
                continue;
            }

            previousAction = action;
            previousheading = atan2(previousAction.x - pstart.x, previousAction.y - pstart.y);
            //s += path[pathindex].toString() + "\n";
            s += action.toString() + "\n";
            sendPath(s);
            communication_With_Controler.cwrite(s);

            if (path_size >= pathindex)
            {

                //int sleeptime = (path[pathindex].otime - previousAction.otime) * 1000;
                //if (sleeptime > 50)
                int sleeptime = 50;
                mtx_path.unlock();

                this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
            }
            else
            {
                communication_With_Controler.cwrite(s);
                //if (send && path_size > pathindex)
                mtx_path.unlock();
                this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        else
        {
            string s = "";
            pstart = current_location;
            s += pstart.toString() + "\n";
            if(!send || cover.size() != 0)
                s += ObjectPar(-1, -1, -1, -1, -1).toString() + "\n";
            else
                s += ObjectPar(-2, -2, -2, -2, -2).toString() + "\n";
            s += "path 0";
            s += "\n" + estimateStart.toString(); //for estimate start
            s += '\0';
            communication_With_Controler.cwrite(s);
            mtx_path.unlock();
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        checkTerminate();
    }
}

void print_map(string file)
{
    if (file != "NOFILE")
    {
        string line;
        ifstream f(file);
        if (f.is_open())
        {
            getline(f, line);
            string w = line;
            getline(f, line);
            string h = line;
            cerr << "EXEUTIVE::START " << w << " " << h << endl;
            cerr << "EXECUTIVE::MAP::" + w + " " + h << endl;
            communication_With_Planner.cwrite("map 1 " + w + " " + h);
            while (getline(f, line))
                communication_With_Planner.cwrite(line);

            f.close();
            return;
        }
    }
    cerr << "EXECUTIVE::MAP::DEFAULT" << endl;
    communication_With_Planner.cwrite("map 1 2000 2000");
    for (int i = 0; i < 2000; i++)
        communication_With_Planner.cwrite("________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________");
}

void read_goal(string goal)
{
    ifstream f(goal);
    if (f.is_open())
    {
        int numofgoal;
        f >> numofgoal;
        double x, y;
        for (int i = 0; i < numofgoal; i++)
        {
            f >> x >> y;
            cover.push_back(point(x, y));
        }
        f.close();
        return;
    }
    else
    {
        cover.push_back(point(10, 10));
        cover.push_back(point(9, 0));
    }
}

int main(int argc, char *argv[])
{
    cout.precision(numeric_limits<float>::digits10 + 2);
    cerr.precision(5);

    communication_With_Planner.set("planner", 1, 1, 0, 0);
    communication_With_Controler.set("controler", 1, 1, 0, 1);

    thread thread_for_controller(thread([=] { sendAction(); }));
    thread thread_for_UDVOBS(thread([=] { requestWorldInformation(); }));

    communication_With_Planner.cwrite("Start");
    communication_With_Planner.cwrite("max speed 2.5");
    communication_With_Planner.cwrite("max turning radius 8");
    bool map = false, goal = false;

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-m"))
        {
            if (i + 1 < argc)
            {
                map = true;
                print_map(argv[i + 1]);
            }
        }
        else if (!strcmp(argv[i], "-g"))
        {
            if (i + 1 < argc)
            {
                goal = true;
                read_goal(argv[i + 1]);
            }
        }
    }

    if (!map)
    {
        string file1 = "NOFILE";
        print_map(file1);
    }
    if (!goal)
    {
        cover.push_back(point(10, 10));
        cover.push_back(point(9, 0));
    }

    int size = cover.size();
    communication_With_Planner.cwrite("path to cover " + to_string(size));
    for (point p : cover)
        communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));

    char done[100];
    communication_With_Planner.cread(done, 100);
    requestPath();

    thread_for_controller.join();
    thread_for_UDVOBS.join();
    return 0;
}
