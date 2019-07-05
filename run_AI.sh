#!/bin/bash

rosrun roboteam_ai Roboteam_AI
while [ $? -ne 0 ]; do
  notify-send -u critical "AI" "AI crashed!"
  rosrun roboteam_ai Roboteam_AI
done