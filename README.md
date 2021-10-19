MIPT center of cognitive modeling


[Apollo](http://apollo.auto) is a high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.

Added special maps to modules/map/data:
* NKB
* NKB_cutted (long straight road with two lanes)
* highway (long highway road with many lanes)

the vehicle and mode are chosen by default "Lincoln2017MKZ LGSVL" and "Mkz Lgsvl".
to delete this automatic choose, comment the following two lines in /modules/dreamview/backend/hmi/hmi_worker.cc :
* ChangeVehicle("Lincoln2017MKZ LGSVL");
* ChangeMode("Mkz Lgsvl");
the same way, the map could be chosen automatically by adding:
* ChangeMap("NKB Cutted");

to run the scenarios of overtaking an obstacle automatically(choosing the map, turnning simcontrol, publishing the obstacles on perception module):
* 1. turn on simcontrol automatically , by uncommenting the following line in /modules/dreamview/backend/sim_control/sim_control.cc
*          Start();
* 2. choose the map automatically, by uncommenting the following line and define the needed map in /modules/dreamview/backend/hmi/hmi_worker.cc:
*          ChangeMap("highway");
* 3. change map_type in PythonAPI/script/publish_obstacles.py to match the map you have chosen in function ChangeMap()
* 4.  $ ./scripts/bootstrap_simcontrol.sh 


you can run the scenarios of overtaking an obstacle manually without steps 1, 2, and 3. by:
* 1. $ ./scripts/bootstrap.sh 
* 2. choose Map, Vehicle, Mode.
* 3. turn on simcontrol mode
* 4. turn on planning, prediction and routing modules.
* 5. $ ./scripts/pub_obstacles.sh

to run scenarios manually by a python script:

* 1. in docker, install needed packages:
	** $pip install keyboard
	** $apt-get update
	** $apt-get install -y kmod kbd
* 2. in docker:
	** $source cyber/setup.bash
	** $source scripts/apollo_base.sh
	** $python3 PythonAPI/scripts/auto_run.py
* 3. to stop running the script and stop recording press q and wait for some time.


* **Software Overview - Navigation Mode**

![image alt text](docs/demo_guide/images/Apollo_3_5_software_architecture.png)


