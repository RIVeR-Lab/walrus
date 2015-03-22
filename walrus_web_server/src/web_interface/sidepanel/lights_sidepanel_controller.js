angular.module("app").controller("LightsSidepanelCtrl", function( $scope, roslib ) {
    var front_pub = roslib.advertise("front_led", "std_msgs/Float64");
    $scope.$watch("front", function(value) {
	if(value) {
	    front_pub.publish({data: 1.0});
	}
	else {
	    front_pub.publish({data: 0.0});
	}
    });

    var back_pub = roslib.advertise("back_led", "std_msgs/Float64");
    $scope.$watch("back", function(value) {
	if(value) {
	    back_pub.publish({data: 1.0});
	}
	else {
	    back_pub.publish({data: 0.0});
	}
    });

    var bottom_pub = roslib.advertise("bottom_led", "std_msgs/Float64");
    $scope.$watch("bottom", function(value) {
	if(value) {
	    bottom_pub.publish({data: 1.0});
	}
	else {
	    bottom_pub.publish({data: 0.0});
	}
    });
});
