#!/bin/sh

rostopic pub -r 3 /ros2osc/$1 $2 "$3 $4"
# while [ true ] ;
# do
#     for pub in $@ ;
#     do
# 	echo "[simple_pub.sh] rostopic pub -1 `echo $pub | tr \"[;]\" \"[ ]\"`;" ;
# 	rostopic pub -1 `echo $pub | tr "[;]" "[ ]"` ;
#     done ;
# done ;
