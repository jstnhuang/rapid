#!/bin/bash
PKG_PATH=`rospack find rapid_perception`
if [ ! -e $PKG_PATH/test_data ]
then
  mkdir $PKG_PATH/test_data
fi
if [ ! -e $PKG_PATH/test_data/table0.bag ]
then
  wget "https://drive.google.com/uc?export=download&id=0B77PnOCaAq8sWUJNZVB0b1l0cUU" -O `rospack find rapid_perception`/test_data/table0.bag
fi
if [ ! -e $PKG_PATH/test_data/table1.bag ]
then
  wget "https://drive.google.com/uc?export=download&id=0B77PnOCaAq8sSk1DTW4yQmhHSU0" -O `rospack find rapid_perception`/test_data/table1.bag
fi
if [ ! -e $PKG_PATH/test_data/table4.bag ]
then
  wget "https://drive.google.com/uc?export=download&id=0B77PnOCaAq8sNVNHNFpZNnk2aUU" -O `rospack find rapid_perception`/test_data/table4.bag
fi
