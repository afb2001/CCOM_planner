#include <iostream>
#include <string>

int main(int args,char *argv[])
{
    int x = std::stoi(argv[3]),y = std::stoi(argv[4]);
    int w = std::stoi(argv[1]),h = std::stoi(argv[2]);
    std::cout << w << "\n";
    std::cout << h << "\n";
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j <  w; j++)
        {
            if(x == i && y == j)
                std::cout << "@" ;
            else
                std::cout << "-" ;
        }
        std::cout << "\n";
    }
    return 0;
}