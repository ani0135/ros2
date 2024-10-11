File Structure..........
```
turtle_teleop/
├── launch
│   └── launch_two_nodes.py
├── package.xml
├── ReadMe.md
├── resource
│   └── turtle_teleop
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── turtle_teleop
    ├── draw_circles.py
    ├── __init__.py
    └── turtle_tele_op.py
```

turtle_teleop is the package name. it is in ~/ros2_ws/src/ directory

Steps : 
1. `mkdir ~/ros2_ws/src` created 
Initialised the empty workspace using `colcon build` in ~ros2_ws/ directory.

2. created a package named "**turtle_teleop**" using `ros2 pkg create turtle_teleop --build-type ament_python` command

3. Now updated directory will look like below

```
ros2_ws/
└── src/
    └── turtle_teleop/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── turtle_teleop
        └── turtle_teleop/
            └── __init__.py
```

4. Added two files for two nodes
    1. draw_circles.py -> to draw three circles
    2. turtle_tele_op.py -> to control turtle using keayboard

5. Edited setup.py file for entry points of two nodes `tele-op and draw_circles`

6. `colcon build` in **~/ros2_ws** directory to build package

7. `source install/setup.bash` for sourcing newly buold package paths in every terminal where we need new node.

8. `ros2 run turtlesim turtlesim_node` in a terminal for launching single instance of turtlesim

9. `ros2 run turtle_teleop draw_circles` in another terminal for lanching newly created node **draw_circles**. It will draw three circles as shown in the 1 tasks.

10. `ros2 run turtle_teleop tele-op` in another terminal for launching **tele-op** node. It will complete the second question in 1st task.

11. added launch_two_nodes.py file to launch two instance of turtlesim node and then
run following commands and copied this file to **~/ros2_ws/install/turtle_teleop/share/turtle_teleop** as I was facing some error
`ros2 launch turtle_teleop launch_two_nodes.py`  launched two instance of turtlesim
`ros2 run turtle_teleop draw_circles` can be used to control both nodes simultaneously.