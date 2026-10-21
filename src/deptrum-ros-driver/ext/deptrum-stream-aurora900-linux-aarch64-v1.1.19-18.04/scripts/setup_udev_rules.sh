#!/bin/bash -e

echo "Setting-up permissions for Deptrum devices"

sudo cp $(dirname "$0")/99-deptrum-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo service udev reload
sudo service udev restart

echo "udev-rules successfully installed"
