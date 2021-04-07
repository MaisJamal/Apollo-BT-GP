#!/usr/bin/env bash







DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DREAMVIEW_URL="http://localhost:8888"

cd "${DIR}/.."

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/apollo_base.sh"

function start() {
    nohup cyber_launch start modules/drivers/tools/image_decompress/launch/image_decompress.launch & 
    nohup ./scripts/bridge.sh start & 
    ./scripts/monitor.sh start
    ./scripts/dreamview.sh start 
    
    
    if [ $? -eq 0 ]; then
        sleep 2  # wait for some time before starting to check
        http_status="$(curl -o /dev/null -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"
        if [ $http_status -eq 200 ]; then
            echo "Dreamview is running at" $DREAMVIEW_URL
        else
            echo "Failed to start Dreamview. Please check /apollo/data/log or /apollo/data/core for more information"
        fi
    fi
    read -p "You are running lgsvl script. Please choose the following: the mode Mkz lgsvl, the map:Borregas Ave, the vehicle: Lincoln and Press [Enter] key to start all the needed  modules..."
    ./scripts/transform.sh start
    ./scripts/localization.sh start
    sleep 1
    ./scripts/prediction.sh start
    sleep 1
    nohup cyber_launch start modules/planning/launch/planning.launch &
    #./scripts/planning.sh start
    sleep 2
    ./scripts/routing.sh start
    ./scripts/control.sh start
    read -p "Press [Enter] key to run LGSVL scenario..."
    ./scripts/lgsvl_scenario.sh start &
    ./scripts/send_routing_request.sh start &
}

function stop() {
    ./scripts/dreamview.sh stop
    ./scripts/monitor.sh stop
    
    ps -ef | grep cyber_bridge | grep -v grep | awk '{print $2}' | xargs kill
    
    ./scripts/transform.sh stop
    ./scripts/localization.sh stop
    ./scripts/prediction.sh stop
    #./scripts/planning.sh stop
    nohup cyber_launch stop modules/planning/launch/planning.launch
    sleep 1
    ./scripts/routing.sh stop
    ./scripts/control.sh stop
   # ./scripts/lgsvl_scenario.sh stop
    cyber_launch stop modules/drivers/tools/image_decompress/launch/image_decompress.launch  
}

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    start
    ;;
esac
