#!/bin/bash

NAMESPACE=${1:-"scout"}
LAT=${2:-"45.8132095"}
LON=${3:-"16.0391788"}
ALT=${4:-"169.88895940447213"}

ros2 topic pub -r 1 /${NAMESPACE}/mavros/home_position/home mavros_msgs/msg/HomePosition "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
geo:
  latitude: ${LAT}
  longitude: ${LON}
  altitude: ${ALT}
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
approach:
  x: 0.0
  y: 0.0
  z: 0.0" 

