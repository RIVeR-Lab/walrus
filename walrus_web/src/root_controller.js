angular.module("app").controller("RootCtrl",
					      function( $scope, roslib, gamepadService ) {
    var joyPub = roslib.advertise("/joy", "sensor_msgs/Joy");
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
});
