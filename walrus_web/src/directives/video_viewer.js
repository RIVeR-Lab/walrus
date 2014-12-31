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
	},
	templateUrl: "/directives/video_viewer.tpl.html"
    };
});
