var app = angular.module("app", ["ros", "gamepad", "ngRoute", "ngMaterial", "html_templates", "svg_templates"]);
angular.module("html_templates", []);
angular.module("svg_templates", []);

var toolGroups = [
    { name: "WALRUS",
      tools: [
	  {name: "Pod Control", path: "/pod-control",
	  templateUrl: "/tools/pod-control-tool.tpl.html",
	  controller: "PodControlToolCtrl"},
	  {name: "Boom Control", path: "/boom-control",
	  templateUrl: "/tools/boom-control-tool.tpl.html",
	  controller: "BoomControlToolCtrl"}
      ]
    },
    { name: "ROS",
      tools: [
	  {name: "Message Tool", path: "/ros-message-tool",
	   templateUrl: "/tools/ros-message-tool.tpl.html",
	   controller: "RosMessageToolCtrl"},
	  {name: "ROS Log", path: "/ros-log-tool",
	   templateUrl: "/tools/ros-log-tool.tpl.html",
	   controller: "RosLogToolCtrl"}
      ]
    }
];
app.factory("toolGroups", function clientIdFactory() {
    return toolGroups;
});

app.config(function(roslibProvider, gamepadServiceProvider, webrtcRosServiceProvider, $mdThemingProvider, $routeProvider){
    roslibProvider.setRosbridgeWsUrl("ws://"+location.hostname+":9003");
    roslibProvider.setPackageUrl("http://"+location.hostname+":9002/");

    webrtcRosServiceProvider.setSignalingUrl("ws://"+location.hostname+":9001/webrtc");

    gamepadServiceProvider.setPollRate(100);

    $mdThemingProvider.theme("default")
	.primaryPalette("blue")
	.accentPalette("pink");

    $routeProvider.when("/", {
	    template: "WALRUS Rover Tools"
	});
    toolGroups.forEach(function(group){
	group.tools.forEach(function(tool){
	    $routeProvider.when(tool.path, {
		templateUrl: tool.templateUrl,
		controller: tool.controller
	    });
	});
    });
});
