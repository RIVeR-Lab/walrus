angular.module("app").controller("RosMessageToolCtrl",
				 function( $scope, $routeParams, roslib ) {
    $scope.messages = [];
    $scope.history = false;
    $scope.topic = "";
    $scope.type = "";
    var sub = null;

    $scope.clear = function() {
	$scope.messages = [];
    };

    $scope.$watch("history", function(newValue) {
	if(newValue === false) {
	    if($scope.messages.length > 0) {
		$scope.messages = [$scope.messages[$scope.messages.length-1]];
	    }
	}
    });

    var callback = function(msg) {
	if($scope.history) {
	    $scope.messages.unshift(msg);
	}
	else {
	    $scope.messages = [msg];
	}
	$scope.$apply();
    };

    function updateSub() {
	if(sub) {
	    sub.shutdown();
	}
	if($scope.topic.length > 0 && $scope.type.length > 0) {
	    $scope.clear();
	    sub = roslib.subscribe($scope.topic, $scope.type, callback);
	}
	else {
	    sub = null;
	}
    }

    $scope.$watch("topic", function(newValue) {
	updateSub();
    });
    $scope.$watch("type", function(newValue) {
	updateSub();
    });

    $scope.$on("$destroy", function() {
	if(sub) {
	    sub.shutdown();
	}
    });
});
