angular.module("app").controller("RootCtrl",
					      function( $scope, roslib, gamepadService, $interval ) {
    var joyPub = roslib.advertise("web_interface/joy", "sensor_msgs/Joy");

    var lastJoyData = null;
    var publish_joy_data = function() {
	if(lastJoyData) {
	    joyPub.publish(lastJoyData);
	}
    };
    $scope.$watch(function() { return gamepadService.gamepad.lastValidData; }, function (data) {
	if(data) {
	    var currentTime = new Date();
	    var secs = Math.floor(currentTime.getTime()/1000);
            var nsecs = Math.round(1000000000 * (currentTime.getTime() / 1000 - secs));
	    var joyData = {
		header: { stamp: { secs: secs, nsecs: nsecs } },
		axes: data.axes,
		buttons: data.buttons.map(function(button){return button.pressed?1:0;})};
	    lastJoyData = joyData;
	}
	else {
	    lastJoyData = null;
	}
	publish_joy_data();
    });
    $interval(publish_joy_data, 250); // republish joystick data every quarter second

    $scope.settings = {
    };
    $scope.available_video_streams = [
	{ label: "Front Camera", topic: "front_camera/image_raw" },
	{ label: "Back Camera", topic: "back_camera/image_raw" },
	{ label: "Bottom Camera", topic: "bottom_camera/image_raw" },
	{ label: "Boom Camera", topic: "boom/kinect/rgb/image_raw" },
	{ label: "Boom Depth Camera", topic: "boom/kinect/depth/image_raw" }
    ];
    $scope.primary_video_stream = $scope.available_video_streams[0];
    $scope.secondary_video_stream = $scope.available_video_streams[3];
});
