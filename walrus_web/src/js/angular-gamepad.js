var rosModule = angular.module("gamepad", []);

rosModule
    .provider("gamepadService",  {
	pollRate: 100,
	setPollRate: function(pollRate) {
	    this.pollRate = pollRate;
	},
	$get: ["$rootScope", "$interval", function($rootScope, $interval) {
	    var lastTimestamp = null;
	    var lastData = null;
	    $interval(function() {
		var gamepadData = navigator.getGamepads()[0];
		if(gamepadData) {
		    if(lastTimestamp === null) {
			$rootScope.$broadcast("gamepad-connected");
		    }
		    if(gamepadData.timestamp !== lastTimestamp) {
			$rootScope.$broadcast("gamepad-data", gamepadData);
			lastTimestamp = gamepadData.timestamp;
			lastData = gamepadData;
		    }
		}
		else if(lastTimestamp !== null){
		    $rootScope.$broadcast("gamepad-disconnected");
		    lastData = null;
		    lastTimestamp = null;
		}
	    }, this.pollRate);
	    return {
		isConnected: function() { return lastTimestamp !== null; },
		getLastData: function() { return lastData; }
	    };
	}]
    });
