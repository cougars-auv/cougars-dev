#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches FGO localization for comparison against a rosbag
# Use the '-r' flag to record a rosbag
# Use the '-p' flag to play a rosbag

function printInfo {
    # print blue
    echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
    # print yellow
    echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
    # print red
    echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

source ~/coug_ws/install/setup.bash

RECORD_BAG_PATH=""
PLAY_BAG_PATH=""

while getopts ":r:p:" opt; do
    case $opt in
        r)
            RECORD_BAG_PATH="$HOME/scripts/plots/$OPTARG"
            if [ -d "$RECORD_BAG_PATH" ]; then
                printError "Bag already exists: $RECORD_BAG_PATH"
                exit 1
            fi
            ;;
        p)
            PLAY_BAG_PATH="$OPTARG"
            if [ ! -d "$PLAY_BAG_PATH" ]; then
                printError "Bag does not exist: $PLAY_BAG_PATH"
                exit 1
            fi
            ;;
        \?)
            printError "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            printError "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

ARGS=()
if [ -n "$RECORD_BAG_PATH" ]; then
    ARGS+=("record_bag_path:=$RECORD_BAG_PATH")
fi
if [ -n "$PLAY_BAG_PATH" ]; then
    ARGS+=("play_bag_path:=$PLAY_BAG_PATH")
fi

ros2 launch coug_bringup compare.launch.py "${ARGS[@]}"