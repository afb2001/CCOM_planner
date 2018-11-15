#include <iostream>
#include <string>
#include <cmath>

using namespace std;

double getCurrentTime()
{
    //change to clock_gettime
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
int main()
{
    cerr.precision(10);
    cout.precision(10);
    string s;
    cin >> s;
    while(s != "path")
    {
        cin >> s;
    }
    cin >> s;
    cin >> s;
    cin >> s;
    cout << "done" << endl << flush; 
    cerr << "FACKPLANNER:START" << endl;
    double x1 = -1,y1,heading1,speed1,otime1;
    while(true)
    {
        
        cin >> s;
        while(s != "start")
        {
            cin >> s;
        }
        cin >> s;
       
        if(x1 == -1)
        {
            cin >> x1 >>y1 >> heading1 >> speed1 >> otime1;

        }
        

        while(s != "dynamic")
        {
            cin >> s;
        }
        cin >> s;
        int num;
        cin >> num;
        for(int i = 0; i< num; i++)
        {
            cin >> s >> s >> s >> s >> s;
        }

        int n = 0;
        cout << "plan 20" << endl << flush; 
        double x = x1,y = y1,heading = heading1,speed = speed1,otime = otime1;
        double addx = -1.09;
        double addy = 0.702;
        double h = -0.8;
        double h1 = 1.57;
        double h2 = 0;
        double ot = getCurrentTime();
        int count = 0;
        double s = 0;
        int find = 0;
        while(true)
        {
            if(n % 20 == 0)
            {
                addx *= -1;
                h *= -1;
            }
            heading = h;
            // if(n < 20)
            // {
            //     h1 = 1.57;
            // }
            // else
            // {
            //     h1 = 0;
            // }
            //heading = h1;

            // if(n % 40 == 0)
            // {
            //     h2 += M_PI/2;
            //     h2 = fmod(h2,M_PI*2);
            // }
            // heading = h2;


            ++n;
            
            otime += 0.5;
            if(s < 2.5)
                s+= 0.1;
            speed = s;
            
            

            x += sin(heading) * speed * 0.5;
            y += cos(heading) * speed * 0.5;
           
            

            

            if(ot < otime && count ++ < 20)
            {
                
                cout << x << " " << y << " " << heading << " " << speed << " "<< otime << endl << flush;  
                find = 1;
            } else if (find)
            {
                break;
            }

        }



    
    }

}