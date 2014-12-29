var app = angular.module("app", ["ros", "gamepad", "ngMaterial"]);

app.config(["roslibProvider", "gamepadServiceProvider", "webrtcRosServiceProvider",
	    function(roslibProvider, gamepadServiceProvider, webrtcRosServiceProvider){
    roslibProvider.setRosbridgeWsUrl("ws://"+location.hostname+":9003");
    roslibProvider.setPackageUrl("http://"+location.hostname+":9002/");

    webrtcRosServiceProvider.setSignalingUrl("ws://"+location.hostname+":9001/webrtc");

    gamepadServiceProvider.setPollRate(100);
}]);
