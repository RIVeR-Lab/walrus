var rosModule = angular.module("gamepad", []);

rosModule
    .provider("gamepadService",  {
	pollRate: 100,
	setPollRate: function(pollRate) {
	    this.pollRate = pollRate;
	},
	$get: function($rootScope, $interval) {
	    var gamepad = {
		config: {
		    axes: {
			inverted: [true, true, true, true]
		    }
		},
		connected: false,
		valid: false,
		lastRawData: null,
		lastValidData: null
	    };

	    var processRawJoystick = function() {
		var processedData = angular.copy(gamepad.lastRawData);
		processedData.axes = processedData.axes.map(function(value, index) {
		    if(gamepad.config.axes.inverted[index]) {
			return -value;
		    }
		    else {
			return value;
		    }
		});

		if(!gamepad.valid) {
		    if(processedData.axes.every(function(axis){ return axis === 0; })) {
			gamepad.valid = true;
		    }
		}
		if(gamepad.valid) {
		    gamepad.lastValidData = processedData;
		}
	    };

	    var isDifferent = function(data) {
		return gamepad.lastRawData === null || data.timestamp !== gamepad.lastRawData.timestamp;
	    };

	    $interval(function() {
		var gamepadData = navigator.getGamepads()[0];
		if(gamepadData) {
		    if(!gamepad.connected) {
			gamepad.connected = true;
			gamepad.valid = false;
		    }
		    if(isDifferent(gamepadData)) {
			gamepad.lastRawData = angular.copy(gamepadData);
			processRawJoystick();
		    }
		}
		else if(gamepad.connected){
		    gamepad.connected = false;
		    gamepad.valid = false;
		    gamepad.lastRawData = null;
		    gamepad.lastValidData = null;
		}
	    }, this.pollRate);

	    return {
		gamepad: gamepad
	    };
	}
    });
