#!/bin/bash

# chmod +x Test_pwm.sh
# ./Test_pwm.sh

rm -f Test_pwm
gcc -o Test_pwm Test_pwm.cpp -lwiringPi -lpthread
sudo ./Test_pwm