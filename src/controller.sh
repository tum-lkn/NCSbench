#!/bin/bash

# Start the controller for a specified time (TEST_LENGTH_IN_S) with a specified name (NAME_OF_TEST) connecting to a
# specified robot (IP_ADDRESS_OF_ROBOT). The measured data is put into the subfolder measurement of the current
# directory. The script also logs the entire data from the RECORDING_INTERFACE.

if [ $(id -u) != "0" ]
then
	echo "controller.sh must be run as root"
        exit 1
fi

if [ "$#" -ne 4 ]
then
	echo "Usage: controller.sh TEST_LENGTH_IN_S NAME_OF_TEST IP_ADDRESS_OF_ROBOT RECORDING_INTERFACE"
	exit 2
fi

FOLDER_NAME="../benchmark/measurements/$(date +%Y%m%d-%H%M%S)-$1s-$2-measurement"
mkdir -p $FOLDER_NAME
echo "Measurement is generated at $FOLDER_NAME"


trap 'kill -INT -$pid' INT
tcpdump -i $4 -s 65535 -w "$FOLDER_NAME/record.pcap" &
if [ "$(uname)" == "Darwin" ]; then
	gtimeout --signal=SIGINT $1 python3 controller.py --address $3 --measurement_folder $FOLDER_NAME &
else
	timeout --signal=SIGINT $1 taskset -c 0,1 python3 controller.py --address $3 --measurement_folder $FOLDER_NAME &
fi
pid=$!
wait $pid
