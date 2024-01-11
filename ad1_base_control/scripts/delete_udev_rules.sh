#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  albabot"
echo "sudo rm   /etc/udev/rules.d/90-usbserial-albabot.rules"
sudo rm   /etc/udev/rules.d/90-usbserial-albabot.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Please restart computer"
