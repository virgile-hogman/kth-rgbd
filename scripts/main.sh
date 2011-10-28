#!/bin/sh

. ./varenv.sh

cd 
~/Projects/kth-rgbd/bin/rgbd_slam $@
cd --
