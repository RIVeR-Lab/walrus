angular.module("app").controller("PodControlToolCtrl",
				 function( $scope, $routeParams, roslib ) {
    $scope.front_left = 0.0;
    $scope.back_left = 0.0;
    $scope.front_right = 0.0;
    $scope.back_right = 0.0;
});

angular.module("app").directive("podControlToolChannel", function() {
    return {
	scope: {
	    value: "=value",
	    label: "@label",
	    controllerNamespace: "@controllerNamespace"
	},
	controller: function($scope, roslib, $interval) {
	    $scope.min = -3.1415;
	    $scope.max = 3.1415;
	    $scope.enable = false;

	    function state_callback(state) {
		$scope.state = state;
		if(!$scope.enable) {
		    $scope.value = state.process_value;
		}
		$scope.actual = state.process_value;
		$scope.absError = Math.abs(state.error) * 100 / Math.PI;
		$scope.set_point = state.set_point;
	    }

	    var command_pub = roslib.advertise($scope.controllerNamespace+"/command", "std_msgs/Float64");
	    var state_sub = roslib.subscribe($scope.controllerNamespace+"/state", "control_msgs/JointControllerState", state_callback);

	    function update() {
		if($scope.enable) {
		    command_pub.publish({data: $scope.value});
		}
	    }

	    $scope.$watch("value", function(value) {
		if(value < $scope.min) {
		    $scope.value = $scope.min;
		}
		if(value > $scope.max) {
		    $scope.value = $scope.max;
		}
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
