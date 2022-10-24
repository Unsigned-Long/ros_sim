#!/bin/bash

# the package path
PKG_PATH=$(dirname $(dirname $(readlink -f "$0")))

XACRO_NAME=${PKG_PATH}/model/xacro/sim-robot.xacro
URDF_NAME=${PKG_PATH}/model/urdf/sim-robot.urdf
SDF_NAME=${PKG_PATH}/model/sdf/sim-robot.sdf

# xacro to urdf
rosrun xacro xacro ${XACRO_NAME} > ${URDF_NAME}
gz sdf -p ${URDF_NAME} > ${SDF_NAME}