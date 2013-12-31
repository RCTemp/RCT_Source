#!/bin/sh

opt="" ;
skip=0 ;
for _opt in $@ ;
do
    if [ "$skip" -lt "1" ] ;
    then
	skip=1 ;
	echo skip $_opt ;
    else
	opt="$opt $_opt" ;
    fi ;
done ;

echo "java -jar $1 $opt ;"
java -jar $1 $opt ;
