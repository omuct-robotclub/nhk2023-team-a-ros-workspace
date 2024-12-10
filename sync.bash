#!/usr/bin/env bash

TARGET="robotclub@robotclub-latitude"
ssh "$TARGET" 'mkdir -p /home/robotclub/work/ros'
rsync -a . "$TARGET":/home/robotclub/work/ros \
  --info=progress2 \
  --exclude={.vscode,log,build,rosbag2*} \
  --delete \
  --copy-unsafe-links
