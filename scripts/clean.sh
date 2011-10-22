#!/bin/sh

read -p " All output data will be erased. Are you sure? (y/[n]) " ans
if [ "$ans" != "y" ]
then
     exit 
fi

cd ~
sudo rm -f data_out/*.*
cd --
