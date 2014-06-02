#!/bin/sh

# Starts both VREP and Matlab, runs $1 in Matlab, and kills VREP when Matlab
# exits. The Matlab script should take care of starting the simulation.
# For this launch.sh to work as intended, run VREP in
#  "continuous remote API server service".
# See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm

if [ $# != 1 -a $# != 2 ]; then
  echo "Example usage: ./launch.sh youbot_01_traj/youbot_01_traj.m [youbot_01_traj/youbot_01_traj.ttt]"
  echo "Only works with *relative* paths for now."
  exit 1
fi

matlab_script="${1/.m/}"
vrep_file=
if [ $# = 2 ]; then
  vrep_file="`pwd`/$2"
else
  vrep_file="`pwd`/${matlab_script}.ttt"
fi

# If V-REP is not installed in $HOME/info0948, add a line like this to your
# .bashrc:
#    export VREP_DIR=/path/to/vrep
if [ -n "$VREP_DIR" ]; then
  vrep_dir="$VREP_DIR"
else
  if [ "`uname`" = "Darwin" ]; then
    vrep_dir="$HOME/info0948/V-REP_PRO_EDU_V3_0_4_Mac"
  else # Linux
    if [ -d "$HOME/info0948/V-REP_PRO_EDU_V3_0_4_Linux" ]; then 
      vrep_dir="$HOME/info0948/V-REP_PRO_EDU_V3_0_4_Linux"
    else
      vrep_dir="$HOME/info0948/V-REP_PRO_EDU_V3_0_4_64_Linux"
    fi
  fi
fi

if [ \! -d "$vrep_dir" ]; then
  echo "V-REP dir not found."
  echo "Add a line like this to your .bashrc:"
  echo "  export VREP_DIR=/path/to/vrep/directory"
  exit 1
fi

# Cancel annoying alert about crash in previous session.
if [ "`uname`" = "Darwin" ]; then
  rm -f "$vrep_dir/vrep.app/Contents/MacOS/system/settings.dat"
else # Linux
  rm -f "$vrep_dir/system/settings.dat"
fi

# Warning: prone to race conditions. You may need to increase the sleep time
# if matlab starts quicker than vrep.
if [ "`uname`" = "Darwin" ]; then
  "$vrep_dir/vrep.app/Contents/MacOS/vrep" "$vrep_file" &
else # Linux
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$vrep_dir
	export LD_LIBRARY_PATH
	$vrep_dir/vrep "$vrep_file" &
fi
VREP_PID=$!
sleep 1
matlab -nodesktop -nosplash -r "run('${matlab_script}'); "
#kill -9 $VREP_PID

# Cancel annoying alert about crash in previous session.
if [ "`uname`" = "Darwin" ]; then
  rm -f "$vrep_dir/vrep.app/Contents/MacOS/system/settings.dat"
else # Linux
  rm -f "$vrep_dir/system/settings.dat"
fi
