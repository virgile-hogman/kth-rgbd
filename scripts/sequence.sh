#!/bin/sh

. ./varenv.sh

if [ ! -d $DIR_PROD ]; then
	mkdir -p $DIR_PROD
	if [ ! -d $DIR_PROD ]; then
		echo "Impossible to create directory '$DIR_PROD'"
		exit
	fi
fi
if [ ! -d $DIR_FRAMES ]; then
	echo "Impossible to find directory '$DIR_FRAMES'"
	exit
fi

read -p " All data in '$DIR_PROD' will be erased. Are you sure? (y/[n]) " ans
if [ "$ans" != "y" ]
then
     exit 
fi

rm -vf $DIR_PROD/*.bmp

cd ..
bin/kth_rgbd -s $@
cd --
