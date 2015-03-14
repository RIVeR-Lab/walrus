angular.module("app").controller("RosMessageToolCtrl",
				 function( $scope, $routeParams, roslib, $q ) {
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
	roslib.callService("/rosapi/topic_type", "rosapi/TopicType", {topic: newValue})
	    .then(function(result) {
		if(result.type.length > 0) {
		    $scope.type = result.type;
		}
	    });
    });
    $scope.$watch("type", function(newValue) {
	updateSub();
    });

    $scope.getMatches = function(searchText) {
	var deferred = $q.defer();
	roslib.callService("/rosapi/topics", "rosapi/Topics", {})
	    .then(function(result) {
		deferred.resolve(result.topics.filter(function(topic){
		    return topic.indexOf(searchText) >= 0;
		}));
	    }, function(error) {
		deferred.reject(error);
	    });
	return deferred.promise;
    };

    $scope.$on("$destroy", function() {
	if(sub) {
	    sub.shutdown();
	}
    });
});
