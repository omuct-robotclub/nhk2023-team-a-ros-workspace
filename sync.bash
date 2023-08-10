#!/usr/bin/env bash

rsync -a . robotclub-e10:/home/robotclub/work/ros \
  --info=progress2 \
  --exclude={.vscode,log,build} \
  --delete \
  --copy-unsafe-links
