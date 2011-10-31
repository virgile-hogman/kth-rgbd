#!/bin/sh

. ./varenv.sh

if [ ! -d $DIR_PROD ]; then
	echo "Directory '$DIR_PROD' not found."
	exit
fi

read -p "All data in '$DIR_PROD' will be erased. Are you sure? (y/[n]) " ans
if [ "$ans" != "y" ]
then
     exit 
fi

rm -vf $DIR_PROD/*.*

echo "Clean complete."
