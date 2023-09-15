#!/usr/bin/env bash

rsync -a . robotclub-latitude:/home/robotclub/work/ros \
  --info=progress2 \
  --exclude={.vscode,log,build,rosbag2*} \
  --delete \
  --copy-unsafe-links
