#!/bin/bash

# chmod +x Test_pwm.sh
# ./Test_pwm.sh

gcc -o Test_pwm Test_pwm.cpp -lwiringPi
sudo ./Test_pwm