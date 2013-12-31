#!/bin/sh

while [ true ] ;
do
    for pub in $@ ;
    do
	echo "[simple_pub.sh] rostopic pub -1 $pub ;" ;
	rostopic pub -1 `echo $pub | tr "[;|]" "[ ]"` ;
    done ;
done ;
