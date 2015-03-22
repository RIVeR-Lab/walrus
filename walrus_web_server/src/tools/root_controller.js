angular.module("app").controller("RootCtrl",
				 function( $scope, $mdSidenav, $location, toolGroups, roslib ) {
    $scope.ros = {
	connected: false,
	message: "ROS Disconnected"
    };

    $scope.$location = $location;
    $scope.toolGroups = toolGroups;

    $scope.toggleSidenav = function() {
	$mdSidenav("left").toggle();
    };


    $scope.$on("ros-connection", function() {
	$scope.ros.connected = true;
	$scope.ros.message = "ROS Connected";
    });
    $scope.$on("ros-error", function(/*error*/) {
	$scope.ros.connected = false;
	$scope.ros.message = "ROS Error";
    });
    $scope.$on("ros-close", function() {
	$scope.ros.connected = false;
	$scope.ros.message = "ROS Disconnected";
    });
});
