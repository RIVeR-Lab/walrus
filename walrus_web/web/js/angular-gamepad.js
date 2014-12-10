var rosModule = angular.module("gamepad", []);

rosModule
    .provider('gamepadService',  {
	$get: ['$rootScope', '$interval', function($rootScope, $interval) {
	    var lastTimestamp = null;
	    var lastData = null;
	    console.log('Created gamepad service');
	    var pollTimer = $interval(function() {
		var gamepadData = navigator.getGamepads()[0];
		if(gamepadData) {
		    if(lastTimestamp === null) {
			$rootScope.$broadcast('gamepad-connected');
		    }
		    if(gamepadData.timestamp !== lastTimestamp) {
			$rootScope.$broadcast('gamepad-data', gamepadData);
			lastTimestamp = gamepadData.timestamp;
			lastData = gamepadData;
		    }
		}
		else if(lastTimestamp !== null){
		    $rootScope.$broadcast('gamepad-disconnected');
		    lastData = null;
		    lastTimestamp = null;
		}
	    }, 100);
	    return {
		isConnected: function() { return lastTimestamp !== null; },
		getLastData: function() { return lastData; }
	    };
	}]
    });
