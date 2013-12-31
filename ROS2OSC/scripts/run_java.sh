#!/bin/sh

opt="" ;
skip=0 ;
for _opt in $@ ;
do
    if [ "$skip" -lt "1" ] ;
    then
	skip=1 ;
	echo "[run java] skip $_opt" ;
    else
	opt="$opt $_opt" ;
    fi ;
done ;

echo "[run java] java -jar $1 $opt ;"
java -jar $1 $opt ;
