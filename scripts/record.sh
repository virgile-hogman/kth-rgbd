#!/bin/sh

read -p " All data will be erased in folder gen. Are you sure? (y/[n]) " ans
if [ "$ans" != "y" ]
then
     exit 
fi

cd ~
sudo rm -f data_out/*.bmp
sudo rm -f data_gen/*
sudo ~/Projects/kth-rgbd/bin/rgbd_slam -r $@
cd --
