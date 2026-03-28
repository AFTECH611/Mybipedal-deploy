#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

for i in 0 1 2 3; do
  ip link set down can$i
  ip link set can$i type can bitrate 1000000 loopback off
  ip link set up can$i
done