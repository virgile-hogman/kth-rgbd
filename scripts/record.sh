#!/bin/sh

. ./varenv.sh

echo "Recording new sequence..."

if [ ! -d $DIR_PROD ]; then
	mkdir -p $DIR_PROD
	if [ ! -d $DIR_PROD ]; then
		echo "Impossible to create directory '$DIR_PROD'"
		exit
	fi
fi
if [ ! -d $DIR_FRAMES ]; then
	mkdir -p $DIR_FRAMES
	if [ ! -d $DIR_FRAMES ]; then
		echo "Impossible to create directory '$DIR_FRAMES'"
		exit
	fi
fi

if [ "`ls -1 $DIR_PROD | wc -l`" -gt "0" ]; then
	echo "All data will be erased in '$DIR_PROD'"
fi
NFRAMES=`ls -1 $DIR_FRAMES | wc -l`
if [ "$NFRAMES" -gt "0" ]; then
	echo "All FRAMES will be erased in '$DIR_FRAMES' : found $NFRAMES files!"
fi
read -p "ARE YOU SURE? (y/[n]) " ans
if [ "$ans" != "y" ]
then
	exit
fi

rm -f $DIR_PROD/*
rm -f $DIR_FRAMES/*.bmp

cd ..
bin/kth_rgbd -r $@
cd --
