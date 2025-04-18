# install opencamlib lib from source (recommended)
cd ~/Downloads
git clone https://github.com/aewallin/opencamlib
cd opencamlib
mkdir build && cd build
cmake .. -D BUILD_CXX_LIB="ON"
make -j4
sudo make install

# install noether lib from source (recommended)
mkdir -p ~/noether_ws/src
cd ~/noether_ws
vcs import src < dependencies.repos
rosdep install --from-paths src -iry
colcon build
