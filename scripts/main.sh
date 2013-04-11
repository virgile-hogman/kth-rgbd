#!/bin/sh

. ./varenv.sh

cd ..
bin/kth_rgbd $@
cd --
