#!/bin/sh

./calibration img_right.txt calib_right.yaml
./calibration img_front.txt calib_front.yaml
./calibration img_left.txt calib_left.yaml
./calibration img_rear.txt calib_rear.yaml

cp ./calib_right.yaml ../data/img/calib_right.yaml
cp ./calib_front.yaml ../data/img/calib_front.yaml
cp ./calib_left.yaml ../data/img/calib_left.yaml
cp ./calib_rear.yaml ../data/img/calib_rear.yaml
