#!/usr/bin/env bash

scp ./configuration.nix robotclub-latitude:~/configuration.nix
ssh robotclub-latitude 'sudo cp ~/configuration.nix /etc/nixos/configuration.nix'
ssh robotclub-latitude 'sudo nixos-rebuild switch'
