#!/bin/bash

cd ./ui/
python3 ui.py & 
cd ../control/
python3 control.py & 
cd ../transmitter/
python3 transmitter.py & 
cd ../drive_message/
python3 drive_message.py & 
cd ../planning/
python3 planning.py