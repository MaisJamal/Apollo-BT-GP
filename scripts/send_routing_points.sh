#!/usr/bin/env bash





DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


cd "${DIR}/.."

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/apollo_base.sh"


#dynamic obstacle in Borregas ave
#python3 PythonAPI/scripts/replay_perception.py PythonAPI/scripts/demo1_onroad_vehicle.json  $@


#my code for publishing 
python3 PythonAPI/scripts/sending_routing_request.py $@







