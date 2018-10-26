# CCOM_planner
Go planner for CCOM ASV

## Requirements
I'm using go version <a href="https://golang.org/dl/">go1.10.3</a> and gccgo (GCC) 8.2.1, so those (or newer) should work for you.
The Python files can run under both python 2.7 and 3.6.

## Instructions
To use, clone the repository and change directories to the root of the project. 
There are scripts to build and run the planner but they just call <code>go build main.go</code> and <code>./main</code> respectively.
</br></br>
There are also a script locates at the root dirctory which can build and run the whole system include simulator, executive and planner. To build, run the build.sh script <code>./build.sh</code>. And to run, run the run.sh script with optional flags -map with grid map file, -goal with goal file and -nobs with number indicates the number of dynamic obstacles. <code>./run.sh [-map gridmap] [-goal goalfile] [-nobs number_of_dobs]</code></br></br>
The sample files of gridmap and goalfile can be found under the sample directory locate at root dirctory, an example running script will be <code> ./run.sh -map sample/test1 -goal sample/goal -nobs 50 </code>
