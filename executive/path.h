#ifndef __PATH_H__
#define __PATH_H__

#include "ObjectPar.h"
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <mutex>

using namespace std;

class Path
{

  public:
    // Default constructor

    Path()
    {
        pathindex = 0;
        current = next_start = ObjectPar(0);
        action = ObjectPar(-1);
    };

    ~Path(){};

    void replacePath(ObjectPar &current)
    {
        if (newpath.size() > 1)
        {
            path.clear();
            pathindex = 0;

            double angle = atan2(current.y - next_start.y, current.x - next_start.x);
            double displacement = (current.otime - next_start.otime) * current.speed;
            double diffx = current.x + displacement * cos(angle) - next_start.x;
            double diffy = current.y + displacement * sin(angle) - next_start.y;
            for (auto i : newpath)
                if (i.otime > current.otime)
                    path.emplace_back(i.x + diffx, i.y + diffy, i.heading, i.speed, i.otime);
        }
        newpath.clear();
        
    };
    //lock this with update info
    void findStart()
    {
        mtx_path.lock();
        int path_size = path.size();
        bool find = false, visit = true;
        ObjectPar current_loc = current;
        double time_1 = current_loc.otime + 1, time_4 = current_loc.otime + 4;
        replacePath(current_loc);

        if (path_size > 1 && pathindex < path_size)
        {
            for (int i = pathindex; i < path_size; i++)
            {
                if (path[i].otime > time_1 && i != 0 && visit)
                {
                    next_start.set(path[i].x, path[i].y, path[i].heading, path[i].speed, time_1);
                    visit = false;
                }
                else if (path[i].otime > time_4 && i != 0)
                {
                    double predictHead = fmod(path[i].heading + 10000 * M_PI, 2 * M_PI);
                    action.set(path[i].x, path[i].y, predictHead, path[i].speed, path[i].otime);
                    find = true;
                    break;
                }
            }
        }

        if (!find)
        {
            if (visit)
            {
                next_start.setEstimate(1, current_loc);
            }
            action = ObjectPar(-1);
        }
        mtx_path.unlock();
    };

    //below for reading the path from planner
    void update_newpath(char currentString[], double &bound)
    {
        sscanf(currentString, "%lf %lf %lf %lf %lf\n", &tempx, &tempy, &tempheading, &tempspeed, &temptime);
        if (bound < temptime)
            newpath.emplace_back(tempx, tempy, tempheading, tempspeed, temptime);
    };


    //below for dynamic obs update
    int update_dynamic_obs(char ObsString[], int byte, int i)
    {
        if (dyamic_obstacles.size() <= i)
            dyamic_obstacles.emplace_back();
        if (sscanf(ObsString + byte, "%d,%lf,%lf,%lf,%lf,%lf\n%n", &dummyindex, &dyamic_obstacles[i].x, &dyamic_obstacles[i].y, &dyamic_obstacles[i].speed, &dyamic_obstacles[i].heading, &dyamic_obstacles[i].otime, &byteREAD) == 6)
            return byteREAD;
        dyamic_obstacles.pop_back();
        return 0;
    };


    //below for current location update
    void update_current(char currentString[], int byte)
    {
        sscanf(currentString + byte, "%lf,%lf,%lf,%lf,%lf [%d]", &current.x, &current.y, &current.speed, &current.heading, &current.otime, &dummy);
    };

    //below for coverd path update
    void update_covered()
    {
        mtx_cover.lock();
        auto it = cover.begin();
        while (it != cover.end())
        {
            float x = it->x - current.x;
            float y = it->y - current.y;
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

    void add_covered(int x, int y)
    {
        cover.emplace_back(x, y);
    };

    //below construct string for controler
    void construct_path_string(string &s)
    {
        int size = path.size();
        s += "path " + to_string(size - pathindex) + "\n";
        for (int i = pathindex; i < size; i++)
            s += path[i].toString() + '\n';
        s += next_start.toString() + '\0'; //for estimate start
    };

    void sendAction(string &s, int &sleep)
    {
        s = "";
        sleep = 50;
        ObjectPar current_loc = current;
        mtx_path.lock();
        int path_size = path.size();
        if (action.otime > current_loc.otime)
        {
            while (path_size > pathindex && current_loc.otime > path[pathindex].otime)
                pathindex++;
            if (path_size == pathindex)
                return;

            s += current_loc.toString() + "\n";
            s += action.toString() + "\n";
            construct_path_string(s);
        }
        else
        {
            s += current_loc.toString() + "\n";
            s += (cover.size() != 0) ? defaultAction_1 : defaultAction_2;
            s += "\npath 0\n" + next_start.toString() + '\0';
        }
        mtx_path.unlock();
    }

    //below construct the request string
    void get_newcovered(string &s)
    {
        mtx_cover.lock();
        int size = newcover.size();
        s += "newly covered " + to_string(size);
        for (point p : newcover)
            s += "\n" + p.toString();
        newcover.clear();
        mtx_cover.unlock();
    };

    void getDynamicObs(string &s)
    {
        mtx_obs.lock();
        int dynamic_obs_size = dyamic_obstacles.size();
        s += "dynamic obs " + to_string(dynamic_obs_size);
        for (int i = 0; i < dynamic_obs_size; i++)
            s += "\n" + to_string(i) + " " + dyamic_obstacles[i].toString();
        mtx_obs.unlock();
    }

    string construct_request_string()
    {
        string s = "plan\n";
        get_newcovered(s);
        findStart();
        s += "\nstart state " + next_start.toString() + "\n";
        getDynamicObs(s);
        return s;
    }

    //below access method for executive
    const vector<ObjectPar> &getDynamicObs() const
    {
        return dyamic_obstacles;
    };

    const vector<ObjectPar> &getPath() const
    {
        return path;
    };

    const ObjectPar &getNext() const
    {
        return next_start;
    };

    const ObjectPar &getCurrent() const
    {
        return current;
    };

    const ObjectPar &getAction() const
    {
        return action;
    };

    const list<point> &get_covered() const
    {
        return cover;
    };

    //below condition check or lock access
    bool finish()
    {
        if (cover.empty())
        {
            action = ObjectPar(-2);
            return true;
        }
        return false;
    }

    void initialize()
    {
        action = next_start = current;
        next_start.otime += 1;
    }

    void lock_obs()
    {
        mtx_obs.lock();
    };

    void unlock_obs()
    {
        mtx_obs.unlock();
    };

  private:
    vector<ObjectPar> path;
    vector<ObjectPar> newpath;
    vector<ObjectPar> dyamic_obstacles;

    list<point> cover, newcover;

    string defaultAction_1 = ObjectPar(-1).toString();
    string defaultAction_2 = ObjectPar(-2).toString();

    mutex mtx_path, mtx_obs, mtx_cover;

    ObjectPar current, action, next_start;

    int pathindex, dummy, byteREAD, dummyindex;

    double tempx, tempy, tempspeed, temptime, tempheading;

    //function
    double estimate_x(double timeiterval, ObjectPar &object)
    {
        return object.x + timeiterval * sin(object.heading) * object.speed;
    };

    double estimate_y(double timeiterval, ObjectPar &object)
    {
        return object.y + timeiterval * cos(object.heading) * object.speed;
    };
};

#endif