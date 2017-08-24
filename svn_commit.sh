#!/bin/bash


files=$(cat modify.txt | xargs)
#echo ${files}


if [ $# -lt 1 ]
    then
    echo "usage: ./commit.sh \"msg\""
    exit
    fi

#echo $1

svn ci --username liuwenchang -m "$1" ${files}
