### To use these scripts, follow the instructions below:
- Copy the scripts to the root folder of a USB flash drive.
- Log on to the admin account on the RoboRIO by using some SSH terminal such as PuTTY (user: admin, password: <<blank>>)
- Copy the script robotautorunner.sh to the RoboRIO folder /home/lvuser
- Make it executable by "chmod +x /home/lvuser/robotautorunner.sh"
- On RoboRIO, edit the file in /etc/udev/rules.d/???? and add a rule:
    ACTION=="add", ATTR(idVendor)=="0a81", ATTRS{idProduct}=="0101", RUN+="/home/lvuser/robotautorunner.sh"
- Reboot the RoboRIO to make it in effect.
