# install autodiff lib from source (recommended but NOT USED)
cd ~/Downloads
git clone https://github.com/autodiff/autodiff
cd autodiff
mkdir .build && cd .build
cmake .. -DAUTODIFF_BUILD_PYTHON=OFF
make -j4    
sudo make install

# install ginac lib using apt (recommended)
sudo apt install -y \ 
    texinfo \
    libcln-dev \
    libginac-dev \

# install ginac lib from source (optional)
# cd ~/Downloads
# wget https://www.ginac.de/ginac-1.8.8.tar.bz2
# tar -xvf ginac-1.8.8.tar.bz2
# cd ginac-1.8.8
# ./configure
# make
# sudo make install