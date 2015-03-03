var app = angular.module("app", ["ros", "gamepad", "ngMaterial", "html_templates", "svg_templates"]);
angular.module("html_templates", []);
angular.module("svg_templates", []);

app.config(function(roslibProvider, gamepadServiceProvider, webrtcRosServiceProvider, $mdThemingProvider){
    roslibProvider.setRosbridgeWsUrl("ws://"+location.hostname+":9003");
    roslibProvider.setPackageUrl("http://"+location.hostname+":9002/");

    webrtcRosServiceProvider.setSignalingUrl("ws://"+location.hostname+":9001/webrtc");

    gamepadServiceProvider.setPollRate(100);

    $mdThemingProvider.definePalette("white", {
	"0": "#ffffff",
	"50": "#ffffff",
	"100": "#ffffff",
	"200": "#ffffff",
	"300": "#ffffff",
	"400": "#ffffff",
	"500": "#ffffff",
	"600": "#ffffff",
	"700": "#ffffff",
	"800": "#ffffff",
	"900": "#ffffff",
	"1000": "#ffffff",
	"A100": "#ffffff",
	"A200": "#ffffff",
	"A400": "#ffffff",
	"A700": "#ffffff",
	"contrastDefaultColor": "dark",
	"contrastLightColors": "600 700 800 900"
    });

    $mdThemingProvider.theme("sidepanel-tabs")
	.primaryPalette("white");
    $mdThemingProvider.theme("default")
	.primaryPalette("blue")
	.accentPalette("deep-orange")
	.backgroundPalette("blue-grey");
});
