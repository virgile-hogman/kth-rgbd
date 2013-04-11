#!/bin/sh

. ./varenv.sh

DIR_SOURCE=$DIR_ARCHIVE/$1
DIR_TARGET=$DIR_PROD

echo "Restoring results from '$DIR_SOURCE'"
if [ ! -d $DIR_SOURCE ]; then
	echo "Directory '$DIR_SOURCE' not found."
	exit
fi
if [ ! -d $DIR_TARGET ]; then
	mkdir -p $DIR_TARGET
	if [ ! -d $DIR_TARGET ]; then
		echo "Impossible to create directory '$DIR_TARGET'"
		exit
	fi
else
	read -p "All data in '$DIR_TARGET' will be erased. Are you sure? (y/[n]) " ans
	if [ "$ans" != "y" ]
	then
		exit 
	fi
fi

echo "Cleaning $DIR_TARGET..."
rm -f $DIR_TARGET/*.*

cd $DIR_SOURCE 
cp -v *.g2o *.dat *.pcd *.log $DIR_TARGET 

echo "Restore complete."
