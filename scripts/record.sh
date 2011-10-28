#!/bin/sh

. ./varenv.sh

echo "Recording new sequence..."

if [ ! -d $DIR_PROD ]; then
	echo $DIR_PROD "not found."
	exit
fi
if [ ! -d $DIR_FRAMES ]; then
        echo $DIR_FRAMES "not found."
        exit
fi

NFRAMES=`ls -1 $DIR_FRAMES | wc -l`

echo "All data will be erased in [$DIR_PROD]"
if [ "$NFRAMES" -gt "0" ]; then
	echo "All FRAMES will be erased in [$DIR_FRAMES] : found $NFRAMES files"
fi
read -p "ARE YOU SURE? (y/[n]) " ans
if [ "$ans" != "y" ]
then
     exit
fi

rm -f $DIR_PROD/*
rm -f $DIR_FRAMES/*.bmp

cd
~/Projects/kth-rgbd/bin/rgbd_slam -r $@
cd --
