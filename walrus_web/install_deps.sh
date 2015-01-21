#!/usr/bin/env bash

sudo add-apt-repository -y ppa:chris-lea/node.js
sudo apt-get update
sudo apt-get install -qq nodejs
sudo npm install -g bower grunt grunt-cli
# Need to clean cache to prevent errors when running npm as non-root user
sudo npm cache clean
