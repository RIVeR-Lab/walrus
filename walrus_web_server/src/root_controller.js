angular.module("app").controller("RootCtrl",
					      function( $scope, roslib, gamepadService ) {
    var joyPub = roslib.advertise("/walrus/web_interface/joy", "sensor_msgs/Joy");
    $scope.$on("gamepad-data", function(ev, data) {
	var currentTime = new Date();
	var secs = Math.floor(currentTime.getTime()/1000);
        var nsecs = Math.round(1000000000 * (currentTime.getTime() / 1000 - secs));
	var joyData = {
	    header: { stamp: { secs: secs, nsecs: nsecs } },
	    axes: data.axes,
	    buttons: data.buttons.map(function(button){return button.pressed?1:0;})};
	joyData.axes[0] = -joyData.axes[0];
	joyData.axes[1] = -joyData.axes[1];
	joyPub.publish(joyData);
    });
    $scope.settings = {
	"touch_joystick" : true
    };
    $scope.available_video_streams = [
	{ label: "Front Camera", topic: "/walrus/front_camera/image_raw" },
	{ label: "Back Camera", topic: "/walrus/back_camera/image_raw" },
	{ label: "Boom Camera", topic: "/walrus/boom/kinect/rgb/image_raw" },
	{ label: "Boom Depth Camera", topic: "/walrus/boom/kinect/depth/image_raw" }
    ];
    $scope.primary_video_stream = $scope.available_video_streams[0];
    $scope.secondary_video_stream = $scope.available_video_streams[3];
});
