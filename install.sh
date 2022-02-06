#!/bin/sh
echo "***********************************************************"
echo "Starting installer for T*"
echo "***********************************************************"

sudo apt-get install libgfortran3 libxss1 python3-pip libtcl8.6 libtk8.6 tk8.6-blt2.5 blt python3-tk libdw1 libbabeltrace1 -y

cd python
sudo pip3 install -r requirements.txt
cd -

cd VarSpeedDubins 
./install_deps.sh
./build.sh
cd ..
#cp VarSpeedDubins/build/libVarSpeed.so c/bin/libVarSpeed.so

# Copy libary that computes all possible transitions for T* algorithm
cp VarSpeedDubins/build/libVarSpeedWind.so c/bin/libVarSpeedWind.so

# Add libary of computes single transition to .so files standart location
# TODO verify it is working on new machine
cp VarSpeedDubins/build/libVarSpeed2WindWoIO.so c/include/libVarSpeed2WindWoIO.so
cp c/include/libVarSpeed2WindWoIO.so /usr/local/lib
chmod 0755 /usr/local/lib/libVarSpeed2WindWoIO.so
ldconfig


cd c/linux
make 
cd -

echo "***********************************************************"
echo "T* install success! execute ./run.sh for an example"
echo "***********************************************************"


