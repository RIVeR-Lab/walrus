angular.module("app").controller("RosLogToolCtrl",
				 function( $scope, $routeParams, roslib ) {
    $scope.messages = [];
    $scope.history = false;
    $scope.topic = "";

    $scope.levels = {
	1: "Debug",
	2: "Info",
	4: "Warn",
	8: "Error",
	16: "Fatal"
    };

    $scope.clear = function() {
	$scope.messages = [];
    };

    var callback = function(msg) {
	$scope.messages.unshift(msg);
	$scope.$apply();
    };

    var sub = roslib.subscribe("/rosout", "rosgraph_msgs/Log", callback);

    $scope.$on("$destroy", function() {
	if(sub) {
	    sub.shutdown();
	}
    });
});
