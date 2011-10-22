#!/bin/sh

cd ~
sudo rm -f data_out/*.bmp
sudo ~/Projects/kth-rgbd/bin/rgbd_slam -s $@
cd --
