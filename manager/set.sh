#!/bin/bash

cd ../simulator/
python3 car_simulator.py & 
cd ../drive_message/
python3 drive_message.py KIAPI_Racing