#!/bin/bash

ndk-build V=1

rm -rf ../../../libs/android/*
cp -rf ../libs/* ../../../libs/android/

rm -rf ../../../sample/android/app/libs/*
cp -rf ../../../libs/android/* ../../../sample/android/ucsvqe/src/main/jniLibs
cp -rf ../../../libs/android/* ../../../sample/android/ucsvqedemo/src/main/jniLibs
