#!/bin/bash
path=`pwd`
cd $path/../models/rmua19_standard_robot1
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2_red
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2_blue
xacro4sdf model.sdf.xacro