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

	    function update() {
		if($scope.enable) {
		    console.log($scope.value);
		    publisher.publish({data: $scope.value});
		}
	    }

	    var publisher = roslib.advertise($scope.controllerNamespace+"/command", "std_msgs/Float64");
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
		publisher.shutdown();
		$interval.cancel(timer);
	    });
	},
	templateUrl: "/tools/pod-control-tool-channel.tpl.html"
    };
});
