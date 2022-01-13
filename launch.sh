#!/bin/bash

case $1 in
gamepad)
  docker-compose -p ar3 up stub_robot fw_native web_interface gamepad
;;
headless)
  docker-compose -p ar3 up stub_robot fw_native web_interface
;;
tribot_headless)
  docker-compose -p ar3 up stub_robot3 fw_native3 web_interface3
;;
webots)
  xhost +
  docker-compose -p ar3 up sim_robot webots_sim fw_native web_interface
;;
*)
  echo "USAGE: $0 <gamepad|headless|tribot_headless|webots>"
;;
esac
