#!/bin/sh

. ./varenv.sh

read -p " All data in [$DIR_PROD] will be erased. Are you sure? (y/[n]) " ans
if [ "$ans" != "y" ]
then
     exit 
fi

rm -vf $DIR_PROD/*.bmp

cd
~/Projects/kth-rgbd/bin/rgbd_slam -s $@
cd --
