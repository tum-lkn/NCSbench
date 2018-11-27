#/bin/bash

# Start the robot and set it up to connect to the IP_ADDRESS_OF_CONTROLLER.
# Unload system modules to save processing capacity.

if [ "$#" -ne 1 ]
then
	echo "Usage: robot.sh IP_ADDRESS_OF_CONTROLLER"
	exit 1
fi

#reduce system load by unloading non essential modules
rmmod legoev3_battery 2>/dev/null
rmmod legoev3_bluetooth 2>/dev/null

python3 robot.py -a $1
