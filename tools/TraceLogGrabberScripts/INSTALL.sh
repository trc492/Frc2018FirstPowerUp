#!/bin/sh
# This is a quick installer for TraceLogGrabberScripts.
# Deliver this through a USB drive with the rest of TraceLogGrabberScripts in the same folder, and run this on the RIO.
cp robot_autorun.sh /home/lvuser/
cp robotautorunner.sh /home/lvuser/
cp 99-usb-grabber.rules /etc/udev/rules.d/
chmod 755 /home/lvuser/robot_autorun.sh
chmod 755 /home/lvuser/robotautorunner.sh
