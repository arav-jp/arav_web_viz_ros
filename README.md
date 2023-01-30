# arav_web_viz_ros

## インストール
```
cd <YOUR_CATKIN_WS>
git clone git@github.com:arav-jp/arav_web_viz_ros.git
git clone git@github.com:arav-jp/arav_edge_control.git -b ryoga_sato/arav_web_viz_ros
git clone git@github.com:arav-jp/hitachi_pkgs.git -b noetic-devel
git clone git@github.com:ros-industrial/ros_canopen.git
catkin_make
```

## CAN設定
https://github.com/RyodoTanaka/can_tools にしたがって設定

```
./peakcan_setup.bash can0 250000
```

## launch
```
roslaunch arav_web_viz_ros start.launch
```

