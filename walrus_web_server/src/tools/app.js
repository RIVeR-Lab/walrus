var app = angular.module("app", ["ros", "gamepad", "ngRoute", "ngMaterial", "html_templates", "svg_templates"]);
angular.module("html_templates", []);
angular.module("svg_templates", []);

app.config(function(roslibProvider, gamepadServiceProvider, webrtcRosServiceProvider, $mdThemingProvider, $routeProvider){
    roslibProvider.setRosbridgeWsUrl("ws://"+location.hostname+":9003");
    roslibProvider.setPackageUrl("http://"+location.hostname+":9002/");

    webrtcRosServiceProvider.setSignalingUrl("ws://"+location.hostname+":9001/webrtc");

    gamepadServiceProvider.setPollRate(100);

    $mdThemingProvider.theme("default")
	.primaryPalette("blue")
	.accentPalette("pink");

    $routeProvider
	.when("/", {
	    template: "WALRUS Rover Tools"
	})
	.when("/ros-message-tool", {
	    templateUrl: "/tools/ros-message-tool.tpl.html",
	    controller: "RosMessageToolCtrl"
	})
	.when("/ros-log-tool", {
	    templateUrl: "/tools/ros-log-tool.tpl.html",
	    controller: "RosLogToolCtrl"
	});
});
