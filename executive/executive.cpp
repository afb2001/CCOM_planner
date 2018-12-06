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
// #include "xtiffio.h"
// #include "geotiffio.h"


using namespace std;

int running = 1;
int request_start = 0;

Path path;
bool debug = true;

bool pause_all = false;

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
        if(debug && pause_all)
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            continue;
        }
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
    char locationString[81920];
    double x, y, heading, speed, otime;
    int index, h, oldbytesRead, bytesRead, count, update;
    while (running)
    {
        read(STDIN_FILENO, locationString, 81920);
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
        else if (!strncmp(locationString, "pause", 5))
        {
            pause_all = true;
        }
        else if (!strncmp(locationString, "start", 5))
        {
            pause_all = false;
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
        if(debug && pause_all)
        {
            this_thread::sleep_for(chrono::milliseconds(1000));
            continue;
        }
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
            string factor = line;
            getline(f, line);
            string w = line;
            getline(f, line);
            string h = line;
            cerr << "EXEUTIVE::START " << w << " " << h << endl;
            cerr << "EXECUTIVE::MAP::" + w + " " + h << endl;
            int width = stoi(w), height = stoi(h);
            path.Maxx = width;
            path.Obstacles = new bool[width * height];
            int hcount = 0;
            communication_With_Planner.cwrite("map " + factor + " " + w + " " + h);
            // while (getline(f, line))
            // {
            //     ++hcount;
            //     communication_With_Planner.cwrite(line);
            //     for (int i = 0; i < line.size(); i++)
            //     {
            //         if (line[i] == '#')
            //             path.Obstacles[path.getindex(i, height - hcount)] = true;
            //         else
            //             path.Obstacles[path.getindex(i, height - hcount)] = false;
            //     }
            // }
            while (getline(f, line))
            {
                ++hcount;
                string s = "";
                char previous = ' ';
                int ncount = 0;
                for (int i = 0; i < line.size(); i++)
                {
                    if (line[i] == '#')
                        path.Obstacles[path.getindex(i, height - hcount)] = true;
                    else
                        path.Obstacles[path.getindex(i, height - hcount)] = false;

                    if (line[i] != previous)
                    {
                        if (i == 0)
                            s += line[i];
                        else
                            s += " " + to_string(ncount);
                        previous = line[i];
                    }
                    ncount += 1;
                }
                communication_With_Planner.cwrite(s);
            }

            f.close();
            return;
        }
    }
    string s = "";
    cerr << "EXECUTIVE::MAP::DEFAULT" << endl;
    communication_With_Planner.cwrite("map 1 2000 2000");
    for (int i = 0; i < 1999; i++)
        s += "_\n";
    s += "_";
    path.Obstacles = new bool[2000 * 2000]{};
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

// void read_tiff(string tiffmap)
// {
//     TIFF *tif = (TIFF *)0;  /* TIFF-level descriptor */
//     GTIF *gtif = (GTIF *)0; /* GeoKey-level descriptor */
//     int versions[3];
//     int cit_length;
//     geocode_t model; /* all key-codes are of this type */
//     char *citation;
//     int size;
//     tagtype_t type;
//     int width,length;

//     /* Open TIFF descriptor to read GeoTIFF tags */
//     tif = XTIFFOpen(tiffmap.c_str(), "r");
//     gtif = GTIFNew(tif);
//     if (tif && gtif)
//     {
//         /* Get the GeoTIFF directory info */
//         GTIFDirectoryInfo(gtif, versions, 0);

//         /* ASCII keys are variable-length; compute size */
//         cit_length = GTIFKeyInfo(gtif, GTCitationGeoKey, &size, &type);
//         if (cit_length > 0)
//         {
//             citation = (char*)malloc(size * cit_length);
//             GTIFKeyGet(gtif, GTCitationGeoKey, citation, 0, cit_length);
//             printf("Citation:%s\n", citation);
//         }

//         /* Get some TIFF info on this image */
//         TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
//         TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &length);

//         /* get rid of the key parser */
//         GTIFFree(gtif);

//         /* close the TIFF file descriptor */
//         XTIFFClose(tif);
//     }
//     cerr << width << " "<<length << " " << cit_length << " " << size << " " << citation<<endl;

//     /* Open GTIF Key parser; keys will be read at this time. */

//     print_map("");
// }

int main(int argc, char *argv[])
{
    string map, goal, tiffmap, boat;
    map = goal = tiffmap = boat = "NOFILE";

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
        else if (!strcmp(argv[i], "-b"))
        {
            if (i + 1 < argc)
                boat = argv[i + 1];
        }
        else if (!strcmp(argv[i], "-debug"))
        {
            path.debug = debug = true;
        }
    }
    cout.precision(numeric_limits<float>::digits10 + 2);
    cerr.precision(5);

    communication_With_Planner.set("planner", 1, 1, 0, 0);
    if (boat != "NOFILE")
        communication_With_Controler.set("controler", 1, 1, 0, 1, boat);
    else
        communication_With_Controler.set("controler", 1, 1, 0, 1);

    thread thread_for_controller(thread([=] { sendAction(); }));
    thread thread_for_UDVOBS(thread([=] { requestWorldInformation(); }));

    communication_With_Planner.cwrite("Start");
    communication_With_Planner.cwrite("max speed 2.5");
    communication_With_Planner.cwrite("max turning radius 8");

    // if (tiffmap != "NOFILE")
    //     read_tiff(tiffmap);
    // else
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
