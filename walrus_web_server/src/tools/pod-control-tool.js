angular.module("app").controller("PodControlToolCtrl",
				 function( $scope, $routeParams, roslib ) {
    $scope.front_left = 0.0;
    $scope.back_left = 0.0;
    $scope.front_right = 0.0;
    $scope.back_right = 0.0;
});

angular.module("app").directive("podControlToolChannel", function() {
    // constants in walrus_pod_controller/PodCommand
    var PodCommand = {
	DISABLED: 0,
	POSITION: 1,
	EFFORT: 2,
	HOLD_POSITION: 3
    };
    return {
	scope: {
	    value: "=value",
	    label: "@label",
	    controllerNamespace: "@controllerNamespace"
	},
	controller: function($scope, roslib, $interval) {
	    $scope.mode = "position";
	    $scope.min = -3.1415;
	    $scope.max = 3.1415;
	    $scope.enable = false;
	    $scope.process_value = 0.0;
	    $scope.Math = Math;

	    function state_callback(state) {
		$scope.state = state;
		if(!$scope.enable) {
		    $scope.value = state.process_value;
		}
		$scope.process_value = state.process_value;
		$scope.actual = state.process_value;
		$scope.command = state.command;
		$scope.set_point = state.set_point;
	    }

	    var command_pub = roslib.advertise($scope.controllerNamespace+"/command", "walrus_pod_controller/PodCommand");
	    var state_sub = roslib.subscribe($scope.controllerNamespace+"/state", "control_msgs/JointControllerState", state_callback);

	    function update() {
		if($scope.value < $scope.min) {
		    $scope.value = $scope.min;
		}
		if($scope.value > $scope.max) {
		    $scope.value = $scope.max;
		}
		if($scope.enable) {
		    if($scope.mode === "position") {
			command_pub.publish({set_point: $scope.value, mode: PodCommand.POSITION});
		    }
		    else {
			command_pub.publish({set_point: $scope.value, mode: PodCommand.EFFORT});
		    }
		}
		else {
		    command_pub.publish({mode: PodCommand.HOLD_POSITION});
		}
	    }

	    $scope.zeroValue = function() {
		$scope.value = 0;
	    };

	    $scope.$watch("mode", function(mode) {
		console.log(mode);
		if(mode === "position") {
		    $scope.min = -3.1415;
		    $scope.max = 3.1415;
		    $scope.value = $scope.process_value;
		}
		else {
		    $scope.value = 0.0;
		    $scope.min = -1.0;
		    $scope.max = 1.0;
		}
		update();
	    });
	    $scope.$watch("value", function(value) {
		update();
	    });

	    var timer = $interval(update, 250);

	    $scope.$on("$destroy", function() {
		state_sub.shutdown();
		command_pub.shutdown();
		$interval.cancel(timer);
	    });
	},
	templateUrl: "/tools/pod-control-tool-channel.tpl.html"
    };
});
