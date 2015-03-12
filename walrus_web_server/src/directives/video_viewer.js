angular.module("app").directive("videoViewer", function() {
    return {
	scope: {
	    stream: "=stream"
	},
	controller: function($scope) {
	    $scope.videoColor = "red";
	    $scope.videoPoster = "img/not-connected.svg";
	    $scope.stateChanged = function(newState) {
		if(newState === "streaming") {
		    $scope.videoColor = "green";
		    $scope.videoPoster = "img/streaming.svg";
		}
		else if(newState === "connected") {
		    $scope.videoColor = "yellow";
		    $scope.videoPoster = "img/connected.svg";
		}
		else {
		    $scope.videoColor = "red";
		    $scope.videoPoster = "img/not-connected.svg";
		}
	    };
	},
	templateUrl: "/directives/video_viewer.tpl.html"
    };
});
