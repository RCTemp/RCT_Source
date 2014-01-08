 #!/bin/sh

if [ $1 -eq 0 ]; then
    sed -i -e "/TURTLEBOT_NAME/d" ~/.bashrc
    echo 'export TURTLEBOT_NAME=teleop_node' >> ~/.bashrc
    echo "teleop node"
    exec bash
elif [ $1 -eq 1 ]; then
    sed -i -e "/TURTLEBOT_NAME/d" ~/.bashrc
    echo 'export TURTLEBOT_NAME=npc1_node' >> ~/.bashrc
    echo "npc1 node"
elif [ $1 -eq 2 ]; then
    sed -i -e "/TURTLEBOT_NAME/d" ~/.bashrc
    echo 'export TURTLEBOT_NAME=npc2_node' >> ~/.bashrc
    echo "npc2 node"
else
    sed -i -e "/TURTLEBOT_NAME/d" ~/.bashrc
    echo "no registered node"
fi
