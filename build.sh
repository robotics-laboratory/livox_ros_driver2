#!/bin/bash

readonly VERSION_ROS2="ROS2"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

# Set working ROS version (ROS2 Iron+ only)
if [ -n "$1" ] && [ "$1" != "ROS2" ]; then
    echo "Invalid argument. Use: ROS2 (or empty)"
    exit 1
fi
echo "ROS version is: ${VERSION_ROS2} (Iron+)"

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

# substitute package.xml
if [ -f package.xml ]; then
    rm package.xml
fi
cp -f package_ROS2.xml package.xml

# build
pushd `pwd` > /dev/null
cd ../../
colcon build
popd > /dev/null

popd > /dev/null
