# Basic commands

## Shell commands

Add this argument to the `colcon build` in python `--symlink-install`,
this to do the build everytime there's a change on the file.
Please note that for this to work,
the python file should be an executable `chmod +x <file.py>`

### Full python build command

`colcon build --packages-select my_py_pkg --symlink-install`

To run use `ros2 run <pkg> <node>`

To run the same node multiple times, you should rename the node using:
`ros run pkg_name node_name --ros-args --remap __node:=<new_name></new>`

To see the list of nodes running, use `node list`
To see th node info, use `node info {node_name}`

In order to graphically see the active nodes, use `rqt`

### Modify a topic freq

```bash
ros2_ws ros2 topic pub -r 10 /robot_news example_interfaces/msg/String "{data: 'Hello from terminal'}"
```

### Rename node pkg_name

`ros2 run pkg_name node_name --ros-args -r __node:=new_node_name`

### Rename topic name

`ros2 run pkg_name node_name --ros-args -r __node:=new_node_name -r old_topic_name:=new_topic_name`

Note that the subscriber needs to match the new topic name
`ros2 run pkg_name node_name --ros-args -r old_topic_name:=new_topic_name`

### Turtlesim

To launch it  
`ros2 run turtlesim turtlesim_node`

To contro it
`ros2 run turtlesim turtle_teleop_key`
