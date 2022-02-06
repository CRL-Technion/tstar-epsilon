#!/usr/bin/env bash
echo "Downloading gcc-6-base" 
cd /tmp/
wget http://archive.ubuntu.com/ubuntu/pool/universe/g/gcc-6/gcc-6-base_6.4.0-17ubuntu1_amd64.deb
echo "Downloading libgfortran3"
wget http://archive.ubuntu.com/ubuntu/pool/universe/g/gcc-6/libgfortran3_6.4.0-17ubuntu1_amd64.deb
echo "Installing gcc-6-base"
sudo dpkg -i gcc-6-base_6.4.0-17ubuntu1_amd64.deb
echo "Installing libgfortran3"
sudo dpkg -i libgfortran3_6.4.0-17ubuntu1_amd64.deb
cd -
