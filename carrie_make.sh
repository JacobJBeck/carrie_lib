#!/bin/bash

sudo rm -r build/*
cd build

for i in $(ls ../src); do
	cmake ../src/$i
	make
	rm CMakeCache.txt 
done

sudo rm -r CMakeFiles
rm cmake_install.cmake
rm Makefile
