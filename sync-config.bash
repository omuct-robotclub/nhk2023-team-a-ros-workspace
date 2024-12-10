#!/usr/bin/env bash

TARGET="robotclub@robotclub-latitude"
scp ./configuration.nix "$TARGET":~/configuration.nix
ssh "$TARGET" 'sudo cp ~/configuration.nix /etc/nixos/configuration.nix'
ssh "$TARGET" 'sudo nixos-rebuild switch'
