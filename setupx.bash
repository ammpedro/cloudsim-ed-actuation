# the directory of this file.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# source ros
. /opt/ros/$ROS_DISTRO/setup.bash

# source the project specific stuff (models, plugins)
#. $DIR/../../install/setup.bash
#. $DIR/../../install/share/cloudsim_ed_actuation/setup.sh

# for quick deploy
. $DIR/../../setup.bash
. $DIR/setup.sh

