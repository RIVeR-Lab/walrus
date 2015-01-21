angular.module("app").directive("videoViewer", function($mdDialog) {
    return {
	scope: {
	    stream: "=stream",
	    available_streams: "=availableStreams"
	},
	controller: function($scope) {
	    $scope.showStreamSelectorDialog = function(ev) {
		$mdDialog.show({
		    targetEvent: ev,
		    templateUrl: "/dialogs/video_stream_selector.tpl.html",
		    controller: "VideoStreamSelectorController",
		    locals: { available_streams: $scope.available_streams }
		}).then(function(result) {
		    $scope.stream = result;
		});
	    };
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
