angular.module("app").controller("RootCtrl",
					      function( $scope, roslib, gamepadService, $interval, $mdDialog ) {
    var joyPub = roslib.advertise("web_interface/joy", "sensor_msgs/Joy");

    var lastJoyData = null;
    var publish_joy_data = function() {
	if(lastJoyData) {
	    joyPub.publish(lastJoyData);
	}
    };
    $scope.$watch(function() { return gamepadService.gamepad.lastValidData; }, function (data) {
	if(data) {
	    var joyData = {
		header: { stamp: roslib.time_now() },
		axes: data.axes,
		buttons: data.buttons.map(function(button){return button.pressed?1:0;})
	    };
	    lastJoyData = joyData;
	}
	else {
	    lastJoyData = null;
	}
	publish_joy_data();
    });
    $interval(publish_joy_data, 250); // republish joystick data every quarter second


    var joyControllerStateCallback = function(state) {
	$scope.controller_state = state;
    };
    var joyControllerStateSub = roslib.subscribe("web_interface/joystick_controller/state", "walrus_joystick_controller/JoystickControllerState", joyControllerStateCallback);


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


    var showStreamSelectorDialog = function(ev) {
	return $mdDialog.show({
	    targetEvent: ev,
	    templateUrl: "/web_interface/dialogs/video_stream_selector.tpl.html",
	    controller: "VideoStreamSelectorController",
	    locals: { available_streams: $scope.available_video_streams }
	});
    };

    $scope.showSecondaryStreamSelectorDialog = function(ev) {
	console.log(ev);
	showStreamSelectorDialog(ev).then(function(result) {
	    $scope.secondary_video_stream = result;
	});
    };
    $scope.showPrimaryStreamSelectorDialog = function(ev) {
	console.log(ev);
	showStreamSelectorDialog(ev).then(function(result) {
	    $scope.primary_video_stream = result;
	});
    };
});
