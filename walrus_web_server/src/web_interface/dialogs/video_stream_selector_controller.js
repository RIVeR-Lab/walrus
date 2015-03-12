angular.module("app").controller("VideoStreamSelectorController",
				 function($scope, $mdDialog, available_streams) {
    $scope.available_streams = available_streams;
    $scope.cancel = function() {
	$mdDialog.cancel();
    };
    $scope.answer = function(answer) {
	$mdDialog.hide(answer);
    };
});
