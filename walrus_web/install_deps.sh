#!/usr/bin/env bash

sudo apt-get install npm
sudo npm install -g bower grunt grunt-cli
npm install
bower install

mkdir libs
pushd libs
wget http://cdn.robotwebtools.org/roslibjs/current/roslib.js
wget http://cdn.robotwebtools.org/threejs/r61/three.js
wget http://cdn.robotwebtools.org/threejs/r61/STLLoader.js
wget http://cdn.robotwebtools.org/threejs/r61/ColladaLoader.js
wget http://cdn.robotwebtools.org/ColladaAnimationCompress/0.0.2/ColladaLoader2.js
wget http://cdn.robotwebtools.org/ros3djs/current/ros3d.js
popd
