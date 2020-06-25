#!/usr/bin/env bash

rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link camera_link 50 &

rosrun tf static_transform_publisher 0.0 0.0 0.4 0.0 0.0 0.0 base_link body 100 &
