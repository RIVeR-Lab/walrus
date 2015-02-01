var app = angular.module("app", ["ros", "gamepad", "ngMaterial", "html_templates", "svg_templates"]);
angular.module("html_templates", []);
angular.module("svg_templates", []);

app.config(function(roslibProvider, gamepadServiceProvider, webrtcRosServiceProvider, $mdThemingProvider){
    roslibProvider.setRosbridgeWsUrl("ws://"+location.hostname+":9003");
    roslibProvider.setPackageUrl("http://"+location.hostname+":9002/");

    webrtcRosServiceProvider.setSignalingUrl("ws://"+location.hostname+":9001/webrtc");

    gamepadServiceProvider.setPollRate(100);

    $mdThemingProvider.theme("normal")
	.primaryPalette("blue")
	.accentPalette("deep-purple");
});
