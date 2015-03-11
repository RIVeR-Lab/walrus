angular.module("app").controller("GamepadDialogController",
					   function($scope, $mdDialog, gamepadService) {
    $scope.gamepad = gamepadService.gamepad;

    $scope.Math = window.Math;
    $scope.hide = function() {
	$mdDialog.hide();
    };
    $scope.cancel = function() {
	$mdDialog.cancel();
    };
    $scope.answer = function(answer) {
	$mdDialog.hide(answer);
    };
});
