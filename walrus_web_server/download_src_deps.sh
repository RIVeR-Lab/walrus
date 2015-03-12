#!/usr/bin/env bash

npm install
bower install

[ -d libs ] || mkdir libs
pushd libs
if [ ! -f three.js ]; then wget http://cdn.robotwebtools.org/threejs/r61/three.js;fi
if [ ! -f STLLoader.js ]; then wget http://cdn.robotwebtools.org/threejs/r61/STLLoader.js;fi
if [ ! -f ColladaLoader.js ]; then wget http://cdn.robotwebtools.org/threejs/r61/ColladaLoader.js;fi
if [ ! -f ColladaLoader2.js ]; then wget http://cdn.robotwebtools.org/ColladaAnimationCompress/0.0.2/ColladaLoader2.js;fi
if [ ! -f ros3d.js ]; then wget http://cdn.robotwebtools.org/ros3djs/current/ros3d.js;fi
if [ ! -f roslib.js ]; then wget http://cdn.robotwebtools.org/roslibjs/current/roslib.js;fi
if [ ! -f eventemitter2.js ]; then wget http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.js;fi
popd
