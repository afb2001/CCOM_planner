# CCOM_planner
Go planner for CCOM ASV

## Requirements
I'm using go version <a href="https://golang.org/dl/">go1.10.3</a> and gccgo (GCC) 8.2.1, so those (or newer) should work for you.
The python files run under python 2.7. You'll need at least <code>numpy</code>,<code>bitarray</code> and <code>pygame</code>.

## Installation
You can directly use the <code>go</code> tool to download the project into your <code>GOPATH</code>:
```
  $ go get github.com/afb2001/CCOM_planner
```
You can also clone the respository yourself:
```
  $ mkdir -p $GOPATH/src/github.com/afb2001
  $ cd $GOPATH/src/github.com/afb2001
  $ git clone https://github.com/afb2001/CCOM_planner.git
```
Once you've downloaded the project you can build the source files:

```
  $ cd $GOPATH/src/github.com/afb2001/CCOM_planner
  $ ./build.sh
```

## Running the system
To run the system (executive, controller, planner, shim, and simulator), call the <code>run.sh</code> script, which 
accepts the following flags:
<ul>
  <li><code>-map &lt;grid map file&gt;</code></li>
  <li><code>-goal &lt;goal file&gt;</code></li>
  <li><code>-nobs &ltnumber of dynamic obstacles&gt;</code></li>
</ul>
For example, using sample map and goal files:

```
./run.sh -map sample/test1.map -goal sample/goal.goal -nobs 50
```
Alternatively, one of several pre-configured tests can be run using the <code>-test</code> flag:
```
./run.sh -test sample/sampleTest/ctest4
```
Those test configurations can be found in sample/sampleTest. They consist of four configuration files: 
<code>testname.asv</code>, <code>testname.dynamic</code>, <code>testname.goal</code> and <code>testname.map</code>.
<code>testname.asv</code> sets the starting state of the vehicle. <code>testname.dynamic</code> configures dynamic 
obstacles. <code>testname.goal</code> sets the points to cover. <code>testname.map</code> is the grid-world-style map,
 which sets the static obstacles.

The simulator and planner can also run separately by running <code>shim.py</code> in the <code>executive</code> folder  
<code>./shim.py [-m gridmap] [-g goalfile]</code> and dynamic_obstacle_sim3.py in the simulator folder. 
<code>./dynamic_obstacle_sim3.py [-p plot] [-m gridmap] [-g goalfile] [-nobs number_of_dobs]</code> 
