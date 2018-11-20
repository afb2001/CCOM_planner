#include <iostream>
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
#include <unordered_set>
#include "path.h"
#include "tiffio.h"

using namespace std;

int running = 1;
int request_start = 0;

Path path;

unordered_set<point> Obstacles;

Communitcation communication_With_Planner, communication_With_Controler;

double getCurrentTime()
{
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

bool checkCollision(double cx, double cy, double ex, double ey)
{
    double slope = (ey - cy) / (ex - cx);

    if (((int)ex) == ((int)cx))
    {
        int cux = cx;
        int starty = cy, endy = ey;
        int increment = (ey > cy) ? 1 : -1;
        while (starty != endy)
        {
            if (Obstacles.find(point(cux, starty)) != Obstacles.end())
                return true;
            starty += increment;
        }
    }
    else if (fabs(slope) > 1)
    {
        int increment = (ey > cy) ? 1 : -1;
        int starty = cy, endy = ey;
        double b = (cx * cy - ex * cy) / (cx - ex);
        while (starty != endy)
        {
            if (Obstacles.find(point((int)((starty - b) / slope), starty)) != Obstacles.end())
                return true;
            starty += increment;
        }
    }
    else
    {
        int increment = (ex > cx) ? 1 : -1;
        int startx = cx, endx = ex;
        double b = (cx * cy - ex * cy) / (cx - ex);
        while (startx != endx)
        {
            if (Obstacles.find(point(startx, slope * startx + b)) != Obstacles.end())
                return true;
            startx += increment;
        }
    }
    return false;
}

// fix the moving of start
void requestPath()
{
    FILE *readstream = fdopen(communication_With_Planner.getWpipe(), "r");
    double start, end, time_bound;
    int numberOfState, sleeptime;
    char response[1024];

    while (!request_start)
        this_thread::sleep_for(std::chrono::milliseconds(50));

    path.initialize();

    while (running)
    {
        if (path.finish())
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            running = 0;
            break;
        }

        start = getCurrentTime();
        communication_With_Planner.cwrite(path.construct_request_string());

        fgets(response, sizeof response, readstream);
        if (!strncmp(response, "done", 4))
        {
            running = 0;
            break;
        }

        sscanf(response, "plan %d\n", &numberOfState);

        time_bound = path.getCurrent().otime;

        for (int i = 0; i < numberOfState; i++) // if no new path then keep old path
        {
            fgets(response, sizeof response, readstream);
            path.update_newpath(response, time_bound);
        }

        end = getCurrentTime();
        sleeptime = (numberOfState) ? ((end - start <= 1) ? ((int)((1 - (end - start)) * 1000)) : 0) : 50;

        this_thread::sleep_for(chrono::milliseconds(sleeptime));
        checkTerminate();
    }
}

void requestWorldInformation()
{
    char locationString[8192];
    double x, y, heading, speed, otime;
    int index, h, oldbytesRead, bytesRead, count, update;
    while (running)
    {
        read(STDIN_FILENO, locationString, 8192);
        if (!strncmp(locationString, "Location", 8))
        {
            request_start = 1;
            path.update_current(locationString, 9);
            path.update_covered();
        }
        else if (!strncmp(locationString, "Obstacle", 8))
        {
            oldbytesRead = bytesRead = 9;
            count = 0;
            path.lock_obs();
            do
            {
                update = bytesRead = path.update_dynamic_obs(locationString, bytesRead, count);
                oldbytesRead += bytesRead;
                bytesRead = oldbytesRead;
                ++count;
                
            } while (update);
            path.unlock_obs();
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

void sendAction()
{
    int sleep;
    string send_string;
    while (running)
    {

        path.sendAction(send_string, sleep);
        if (send_string != "")
            communication_With_Controler.cwrite(send_string);
        this_thread::sleep_for(std::chrono::milliseconds(50));
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
            int width = stoi(w), height = stoi(h);
            int hcount = 0;
            communication_With_Planner.cwrite("map 1 " + w + " " + h);
            while (getline(f, line))
            {
                ++hcount;
                communication_With_Planner.cwrite(line);
                for (int i = 0; i < line.size(); i++)
                {
                    if (line[i] == '#')
                    {
                        Obstacles.insert(point(i, height - hcount));
                    }
                }
            }

            f.close();
            return;
        }
    }
    string s = "";
    cerr << "EXECUTIVE::MAP::DEFAULT" << endl;
    communication_With_Planner.cwrite("map 1 2000 2000");
    for (int i = 0; i < 2000; i++)
        s += '-';
    for (int i = 0; i < 2000; i++)
        communication_With_Planner.cwrite(s);
}

void read_goal(string goal)
{
    ifstream f(goal);
    if (goal != "NOFILE" && f.is_open())
    {
        int numofgoal;
        f >> numofgoal;
        double x, y;
        for (int i = 0; i < numofgoal; i++)
        {
            f >> x >> y;
            cerr << "cover " << x << " " << y << endl;
            path.add_covered(x, y);
        }
        f.close();
        return;
    }
    else
    {
        path.add_covered(10, 10);
        path.add_covered(9, 0);
    }
}

void read_tiff(string tiffmap)
{
    tiff *tif = TIFFOpen(tiffmap.c_str(), "rc");  
    unsigned int width,height; 
    if (tif != nullptr)                          
    {
        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);           // uint32 width;
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height); 
        cerr << width << " " << height << endl;

        uint32 npixels=width*height;
        uint32 *raster=(uint32 *) _TIFFmalloc(npixels *sizeof(uint32));

        _TIFFfree(raster);
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

    string map, goal, tiffmap;
    map = goal = tiffmap = "NOFILE";

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-m"))
        {
            if (i + 1 < argc)
                map = argv[i + 1];
        }
        else if (!strcmp(argv[i], "-g"))
        {
            if (i + 1 < argc)
                goal = argv[i + 1];
        }
        else if (!strcmp(argv[i], "-t"))
        {
            if (i + 1 < argc)
                tiffmap = argv[i + 1];
        }
    }

    if (tiffmap != "NOFILE")
        read_tiff(tiffmap);
    else
        print_map(map);
    read_goal(goal);

    communication_With_Planner.cwrite("path to cover " + to_string(path.get_covered().size()));
    for (point p : path.get_covered())
        communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));

    char done[100];
    communication_With_Planner.cread(done, 100);
    requestPath();

    thread_for_controller.join();
    thread_for_UDVOBS.join();
    return 0;
}
