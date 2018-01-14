#!/bin/sh
echo "TRC Library installer (C) Titan Robotics Club 2018"
echo -n "Would you like set up the NAVX and Phoenix libraries? (Y/N)"
read answer
if echo "$answer" | grep -iq "^y" ;then
    echo "Installing NAVX_FRC and CTRE Phoenix dependencies..."
    cp libs/* ~/wpilib/java/current/lib/
    echo "Updating config file..."
    rm ~/wpilib/java/current/ant/build.xml
    cp build.xml ~/wpilib/java/current/ant/
    echo "Finished installing."
else
    echo "Cancelled installation."
fi
