#!/bin/sh

. ./varenv.sh

DIR_TARGET=$DIR_ARCHIVE/$1

echo "Backup results..."

if [ ! -d $DIR_PROD ]; then
	echo "Directory '$DIR_PROD' not found."
	exit
fi
if [ -d $DIR_TARGET ]; then
	read -p "Directory '$DIR_TARGET' already exists. Overwrite? (y/[n])" ans
	if [ "$ans" != "y" ]; then
		exit 
	fi
else
	mkdir -p $DIR_TARGET
	if [ ! -d $DIR_ARCHIVE ]; then
		echo "Impossible to create directory '$DIR_TARGET'"
		exit
	fi
fi

echo "Saving results in '$DIR_TARGET'..."
cp -v $DIR_PROD/*.g2o $DIR_TARGET
cp -v $DIR_PROD/*.dat $DIR_TARGET
cp -v $DIR_PROD/*.pcd $DIR_TARGET
cp -v $DIR_PROD/*.log $DIR_TARGET

echo "Backup complete."
