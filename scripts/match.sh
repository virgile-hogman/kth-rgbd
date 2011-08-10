cd ~
sudo rm -f data_out/*.bmp
sudo ~/Projects/kth-rgbd/bin/rgbd_slam -match $1 $2 $3
cd --
