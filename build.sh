#!/bin/bash

readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_HUMBLE=""

# Set working ROS version (ROS2 only)
if [ "$1" = "ROS2" ] || [ -z "$1" ]; then
    :
elif [ "$1" = "humble" ]; then
    ROS_HUMBLE=${VERSION_HUMBLE}
else
    echo "Invalid argument. Use: ROS2|humble"
    exit
fi
echo "ROS version is: ${VERSION_ROS2}"

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# exit

# substitute the files/folders: package.xml, launch/
if [ -f package.xml ]; then
    rm package.xml
fi
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

# build
pushd `pwd` > /dev/null
cd ../../
colcon build --cmake-args -DHUMBLE_ROS=${ROS_HUMBLE}
popd > /dev/null

# remove the substituted folders/files
rm -rf launch/

popd > /dev/null
