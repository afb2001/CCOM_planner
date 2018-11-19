CC = gcc
CXX = g++

all: executive controler

executive: executive.cpp communication.o
	$(CXX)  executive.cpp communication.o -O2 -o executive -std=c++11 -lpthread

controler: controler.cpp communication.o
	$(CXX)  controler.cpp communication.o -O2 -o controler -std=c++11 -lpthread

communication.o: communication.cpp communication.h
	$(CXX) -c communication.cpp -std=c++11 -O2

path:path1.cpp
	$(CXX)  path1.cpp -O2 -std=c++11 -o planner



clean :
	-rm executive controler communication.o 
