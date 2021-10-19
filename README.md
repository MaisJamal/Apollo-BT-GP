## Apollo-BT-GP

Apollo-BT-GP is an updated version of [Apollo 6.0](https://github.com/ApolloAuto/apollo). A behavior tree was integrated into the planning module to make the decision of overtaking dynamic obstacles with relatively high speeds. The behavior tree's structure was learned using genetic programming algorithm.

## Publications

* Adaptive Maneuver Planning for Autonomous Vehicles using Behavior Tree on Apollo Platform

## Installation

The installation steps are basically the same as Apollo's installation:
```
$ git clone https://github.com/MaisJamal/Apollo-BT-GP.git
$ cd Apollo-BT-GP
Apollo-BT-GP$ bash docker/scripts/dev_start.sh
Apollo-BT-GP$ bash docker/scripts/dev_into.sh
root@in-dev-docker:/apollo# ./apollo.sh build_gpu            //just for the first time, and after making changes to the code

```

## Notes

### Maps 

some maps has been added to modules/map/data:
* NKB
* NKB_cutted (long straight road with two lanes)
* highway (long highway road with many lanes)

### Vehicle and Mode

The vehicle and mode are chosen by default "Lincoln2017MKZ LGSVL" and "Mkz Lgsvl".
to delete this automatic choose, comment the following two lines in /modules/dreamview/backend/hmi/hmi_worker.cc :
* ChangeVehicle("Lincoln2017MKZ LGSVL");
* ChangeMode("Mkz Lgsvl");
By the same way, the map could be chosen automatically by adding:
* ChangeMap("NKB Cutted");

### Running scenario

To run the scenarios of overtaking an obstacle automatically(choosing the map, turnning simcontrol, publishing the obstacles on perception module):
1. turn on simcontrol automatically , by uncommenting the following line in /modules/dreamview/backend/sim_control/sim_control.cc
	```Start();```
2. choose the map automatically, by uncommenting the following line and define the needed map in /modules/dreamview/backend/hmi/hmi_worker.cc:
	```ChangeMap("highway");```
3. change map_type in PythonAPI/script/publish_obstacles.py to match the map you have chosen in function ChangeMap()
4.  ```$ ./scripts/bootstrap_simcontrol.sh ```


You can run the scenarios of overtaking an obstacle manually without steps 1, 2, and 3. by:
1. ```$ ./scripts/bootstrap.sh ```
2. choose Map, Vehicle, Mode.
3. turn on simcontrol mode
4. turn on planning, prediction and routing modules.
5. ```$ ./scripts/pub_obstacles.sh```

To run scenarios manually by a python script:

1. in docker, install needed packages:
	* ```$pip install keyboard```
	* ```$apt-get update```
	* ```$apt-get install -y kmod kbd```
2. in docker:
	* ```$source cyber/setup.bash```
	* ```$source scripts/apollo_base.sh```
	* ```$python3 PythonAPI/scripts/auto_run.py```
3. to stop running the script and stop recording press q and wait for some time.




