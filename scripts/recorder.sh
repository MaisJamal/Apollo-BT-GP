#!/usr/bin/env bash



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "${DIR}/apollo_base.sh"

function start() {
  LOG="/tmp/apollo_record.out"
  today=`date +%Y-%m-%d.%H:%M:%S`
  filename="data/bag/$today.record"
  cyber_recorder record -a -o $filename
}

function stop() {
  pkill -SIGINT -f record
}

#cyber_recorder play -f ./data/bag/***.record



case $1 in
  start)
    start $@
    ;;
  stop)
    stop $@
    ;;
# play)
#   play $@
#   ;;
  *)
    start $@
    ;;
esac
