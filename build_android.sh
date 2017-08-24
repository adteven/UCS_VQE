#!/bin/bash

echo "------------------------------------------------------"
echo "                 Android Build Shell                  "
echo "                2017-08-10 Vinton.Liu                 "
echo "------------------------------------------------------"

curdir=`pwd`
cd ${curdir}

build_all() {
    cd UcsVQE/android/jni
    chmod +x build.sh
    ./build.sh
    cd ${curdir}
}

clean_all() {
    cd UcsVQE/android/jni
    ndk-build clean
    cd ${curdir}
}

if [ "$1" = "all" ]
then
build_all
fi


if [ "$1" = "clean" ]
then
clean_all
rm -rf UcsVQE/android/libs/ UcsVQE/android/obj/
fi
