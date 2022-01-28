
CAV-Game setup steps


Step 1:  ....


Step--: ....
Copy and paste the lines below in your the ~/.bashrc file

# Unreal engine path for carla
export UE4_ROOT=~/UnrealEngine_4.26

# Scenario Runner confgi variables
export CARLA_ROOT=~/CAV-Game
export SCENARIO_RUNNER_ROOT=${CARLA_ROOT}/scenario_runner_0.9.13
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.6-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
