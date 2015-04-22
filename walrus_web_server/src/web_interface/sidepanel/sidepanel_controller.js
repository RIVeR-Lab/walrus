angular.module("app").controller("SidepanelCtrl", function( $scope, roslib ) {
    $scope.tabs = {
	selectedIndex : 0
    };
    $scope.$watch("tabs.selectedIndex", function(newIndex, oldIndex){
	if(newIndex !== oldIndex) {
	    setTimeout(function() { // Need to delay to allow for the tabs to actually change
		$scope.$broadcast("tab-changed", newIndex, oldIndex);
	    }, 100);
	}
    });

    $scope.toDegrees = function(rad) {
	return rad * 180 / Math.PI;
    };

    $scope.pitch = 0;
    $scope.roll = 0;
    var imuCallback = function(imu) {
	$scope.pitch = Math.atan2(imu.linear_acceleration.y, imu.linear_acceleration.z) + Math.PI;
	$scope.roll = Math.atan2(imu.linear_acceleration.x, imu.linear_acceleration.z) + Math.PI;
    };
    var imuSub = roslib.subscribe("imu/data_throttle", "sensor_msgs/Imu", imuCallback);

});
