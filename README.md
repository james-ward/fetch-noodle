# fetch-noodle

## Instructions

### Build on Fetch

Copy the contents of this repository to the Fetch.

There is already a workspace in the Fetch home directory called `noodle_ws`

`ssh -Y fetch@fetch39`
`cd noodle_ws`

Build the packages: `catkin_make`

### Configure runs

Source the overlay: `source devel/setup.bash`

Edit parameters:
`cd fetch_noodle_gui/scripts`
`nano runs.yaml`

Load parameters: `rosparam load runs.yaml`

### Run experiment

Open first ssh session
`ssh -Y fetch@fetch39`
`cd noodle_ws`
`source devel/setup.bash`
`roslaunch fetch_noodle_nav fetch_noodle_nav.launch`

Second ssh session:
`ssh -Y fetch@fetch39`
`cd noodle_ws`
`source devel/setup.bash`
`rosrun fetch_noodle_gui fetch_noodle.py`

