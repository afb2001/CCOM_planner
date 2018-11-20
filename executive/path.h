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

    void replacePath(ObjectPar &current);
    //lock this with update info
    void findStart();

    //below for reading the path from planner
    void update_newpath(char currentString[], double &bound);


    //below for dynamic obs update
    int update_dynamic_obs(char ObsString[], int byte, int i);


    //below for current location update
    void update_current(char currentString[], int byte);

    //below for coverd path update
    void update_covered();

    void add_covered(int x, int y);

    //below construct string for controler
    void construct_path_string(string &s);

    void sendAction(string &s, int &sleep);

    //below construct the request string
    void get_newcovered(string &s);

    void getDynamicObs(string &s);

    string construct_request_string();

    //below access method for executive
    const vector<ObjectPar> &getDynamicObs() const;

    const vector<ObjectPar> &getPath() const;

    const ObjectPar &getNext() const;

    const ObjectPar &getCurrent() const;

    const ObjectPar &getAction() const;

    const list<point> &get_covered() const;

    //below condition check or lock access
    bool finish();

    void initialize();
    void lock_obs();
    void unlock_obs();

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