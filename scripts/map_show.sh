#!/usr/bin/env bash



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "${DIR}/apollo_base.sh"

function start() {
  python modules/tools/mapshow/mapshow.py -m /modules/map/data/borregas_ave/base_map -sl
}

#function stop() {
  
#}





case $1 in
  start)
    start $@
    ;;
#  stop)
#    stop $@
#    ;;
# play)
#   play $@
#   ;;
  *)
    start $@
    ;;
esac
