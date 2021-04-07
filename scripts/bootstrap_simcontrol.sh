#!/usr/bin/env bash



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DREAMVIEW_URL="http://localhost:8888"

cd "${DIR}/.."

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/apollo_base.sh"

function start() {
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
    echo "You are running SimControl script..."
    sleep 1
    ./scripts/prediction.sh start

    sleep 1
    nohup cyber_launch start modules/planning/launch/planning.launch &
  #  ./scripts/planning.sh start
    sleep 1
    ./scripts/routing.sh start
 #   ./scripts/control.sh start
    sleep 2
  #  echo "recording has started ..."
    ./scripts/pub_obstacles.sh
 #   ./scripts/pub_obstacles.sh &
 #   ./scripts/recorder.sh start 

}

function stop() {
    ./scripts/dreamview.sh stop
    ./scripts/monitor.sh stop
    
    ./scripts/prediction.sh stop
   # ./scripts/planning.sh stop
    nohup cyber_launch stop modules/planning/launch/planning.launch
    kill $(pgrep mainboard)
    sleep 2
    ./scripts/routing.sh stop
 #   ./scripts/control.sh stop
  #  ./scripts/recorder.sh stop
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
