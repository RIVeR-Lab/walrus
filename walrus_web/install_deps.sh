#!/usr/bin/env bash

sudo add-apt-repository -y ppa:chris-lea/node.js
sudo apt-get update
sudo apt-get install -qq nodejs
sudo npm install -g bower grunt grunt-cli
# Need to clean cache to prevent errors when running npm as non-root user
sudo npm cache clean
npm install
bower install

[ -d libs ] || mkdir libs
pushd libs
wget http://cdn.robotwebtools.org/roslibjs/current/roslib.js
wget http://cdn.robotwebtools.org/threejs/r61/three.js
wget http://cdn.robotwebtools.org/threejs/r61/STLLoader.js
wget http://cdn.robotwebtools.org/threejs/r61/ColladaLoader.js
wget http://cdn.robotwebtools.org/ColladaAnimationCompress/0.0.2/ColladaLoader2.js
wget http://cdn.robotwebtools.org/ros3djs/current/ros3d.js
popd
