#!/usr/bin/env bash

# This script checks the file multi_run_list.txt in the same directory
# For each of the sequences contained, it will make N tests

### ADAPT THESE PATHS ###

# Set the path where the trajectories should be saved
savedir="/home/semesterproject/data/runs"

# Set the path to the directory of the dataset
datadir="/home/semesterproject/data/tank"

# Absolute path to the semesterprojec directory
projectdir="/home/semesterproject"



# - Make sure the paths to the rosbags and saving locations work out
# - Make sure you use the right parameters and have the pipeline compiled
#   as you want it before running this. Also: the convert_compressed_img node 
#   has to be running in parallel in a separate terminal.
# - The multi_run_list.txt file must not contain any additional whitespace 
#   and end with a newline
N=2
factors=("1.0" "0.1")
# factors=("10.0")

srcdir="${projectdir}/src"

for factor in "${factors[@]}"; do

    echo " --- FACTOR ${factor} --- "

    while IFS= read -r NAME; do

        echo " -- SEQUENCE ${NAME} -- "

        # New directory if necessary
        dirname="${savedir}/${NAME}/sidm_dvlcov${factor/./p}"

        if [ ! -d $dirname ]; then
            echo "New directory"
            mkdir -p $dirname
        else
            echo "Directory already exists"
        fi

        # Make the changes and recompile
        sed -i "8 s/=[^*]*/=${factor}/" "${srcdir}/slam-wrapper-v2/orb_slam3/src/DvlTypes.cc"
        
        xterm -e bash -c "cd "${projectdir}" && pwd && colcon build" &
        BUILDER_PID=$!
        wait $BUILDER_PID

        for (( i=1 ; i<=$N ; i++ ));
        do

            # Start recorder
            xterm -e bash -c "source ${projectdir}/install/setup.bash && ros2 run convert_compressed_img save_trajec" &
            RECORDER_XTERM_PID=$!
            echo "Recorder xterm PID: $RECORDER_XTERM_PID"

            # Start pipeline
            xterm -e bash -c "source ${projectdir}/install/setup.bash && ros2 launch ros2_orb_slam3 stereo.launch.py" &
            PIPELINE_XTERM_PID=$!
            echo "Pipeline xterm PID: $PIPELINE_XTERM_PID"
            sleep 2  # give everything a moment to start up

            # Start rosbag
            xterm -e bash -c "source ${projectdir}/install/setup.bash && ros2 bag play ${datadir}/${NAME}/" &
            PLAYER_XTERM_PID=$!
            echo "Player xterm PID: $PLAYER_XTERM_PID"

            # Wait for rosbag to finish
            wait $PLAYER_XTERM_PID

            echo "Rosbag finished — sending Ctrl-C to recorder process..."

            # Find the child shell inside the recorder xterm
            RECORDER_SHELL_PID=$(pgrep -P $RECORDER_XTERM_PID)

            # Send Ctrl-C to the process group of the recorder shell
            kill -SIGINT -$RECORDER_SHELL_PID

            # Find the name for the new file
            count=$(find $dirname -maxdepth 1 -type f | wc -l)
            next=$((count + 1))
            formatted=$(printf "%02d" "$next")

            sleep 4  # Give the ros node time to save the data to a file, before moving that file to the final location
            mv live_trajec.txt ${dirname}/live_trajec_${formatted}.txt && \

            kill "$PIPELINE_XTERM_PID"

        done

    done < "${projectdir}/src/slam-wrapper-v2/scripts/multi_run_list.txt"

done
