# pixkit-autoware


```
mkdir -p ~/pixcon_ws/src
cd ~/pixcon_ws/src
ros2 pkg create --build-type ament_python pubsub
```

this branch is in `pixcon_ws/src/ ` 


```
sudo apt install can-utils
pip3 install python-can
pip3 install python-can-remote
pip3 install pygame
```


in `pixcon_ws`
```
colcon build
source ~/pixcon_ws/install/setup.bash

ros2 run pubsub mode_publisher
ros2 run pubsub commander
ros2 run pubsub reporter
```
