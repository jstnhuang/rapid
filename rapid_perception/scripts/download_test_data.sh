#!/bin/bash
PKG_PATH=`rospack find rapid_perception`
if [ ! -e $PKG_PATH/test_data ]
then
  mkdir $PKG_PATH/test_data
fi
wget "https://drive.google.com/uc?export=download&id=0B77PnOCaAq8sNVNHNFpZNnk2aUU" -O `rospack find rapid_perception`/test_data/table4.bag
