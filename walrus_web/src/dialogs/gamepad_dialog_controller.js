angular.module("app").controller("GamepadDialogController", ["$scope", "$mdDialog", "gamepadService",
					   function($scope, $mdDialog, gamepadService) {
    $scope.gamepad = {
	connected: gamepadService.isConnected(),
	data: gamepadService.getLastData()
    };

    $scope.$on("gamepad-connected", function() {
	$scope.gamepad.connected = true;
    });
    $scope.$on("gamepad-disconnected", function() {
	$scope.gamepad.connected = false;
    });
    $scope.$on("gamepad-data", function(ev, data) {
	$scope.gamepad.data = data;
    });

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
}]);
