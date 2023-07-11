#!/bin/bash



cd
sudo apt-get install libusb-1.0-0-dev
git clone https://github.com/libuvc/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install

sudo echo 'sudo sh -c "echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}==\"1e4e\", ATTRS{idProduct}==\"0100\", SYMLINK+=\"pt1\", GROUP=\"usb\", MODE=\"666\"' > /etc/udev/rules.d/99-pt1.rules"' >> ~/.bashrc

