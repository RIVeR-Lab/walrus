var app = angular.module('app', ['ngMaterial']);

app.controller('SidepanelCtrl', function( $scope ) {
    $scope.tabs = {
	selectedIndex : 0
    };
});

app.controller('DiagnosticsCtrl', function( $scope, $mdDialog ) {
    $scope.diagnostics = {
	cpu: 0.7,
	memory: 0.5,
	batteries: [0.2, 0.3, 0.6, 0.9],
	batteries_connected: [true, false, true, true],
	pods: [0.5, 0.6, 0.1, 0.0],
	drive: [0.5, -0.3]
    };
    $scope.percentToColorHighBad = function(val){
	if(val > 0.8)
	    return 'red';
	else if(val > 0.6)
	    return 'orange';
	else
	    return 'green';
    };
    $scope.percentToColorLowBad = function(val){
	if(val < 0.2)
	    return 'red';
	else if(val < 0.4)
	    return 'orange';
	else
	    return 'green';
    };

    $scope.Math = window.Math;


    $scope.showControlsLayout = function(ev){
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: 'controls_dialog.html',
	    controller: ControlsDialogController
        });
    };
    $scope.showDiagnostics = function(ev){
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: 'diagnostics_dialog.html',
	    locals: { diagnostics: $scope.diagnostics },
	    controller: DiagnosticsDialogController
        });
    };
    $scope.showGamepadDiagnostics = function(ev){
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: 'gamepad_dialog.html',
	    controller: GamepadDialogController
        });
    };
});

function DiagnosticsDialogController($scope, $mdDialog, diagnostics) {
  $scope.diagnostics = diagnostics;
  $scope.hide = function() {
    $mdDialog.hide();
  };
  $scope.cancel = function() {
    $mdDialog.cancel();
  };
  $scope.answer = function(answer) {
    $mdDialog.hide(answer);
  };
}


function ControlsDialogController($scope, $mdDialog) {
  $scope.hide = function() {
    $mdDialog.hide();
  };
  $scope.cancel = function() {
    $mdDialog.cancel();
  };
  $scope.answer = function(answer) {
    $mdDialog.hide(answer);
  };
}


function GamepadDialogController($scope, $mdDialog, $interval) {
    $scope.Math = window.Math;
    var pollTimer;
    pollTimer = $interval(function() {
	$scope.gamepad = navigator.getGamepads()[0];
	console.log($scope.gamepad);
    }, 100);
    $scope.$on('$destroy', function() {
        $interval.cancel(pollTimer);
    });
    $scope.hide = function() {
	$mdDialog.hide();
    };
    $scope.cancel = function() {
	$mdDialog.cancel();
    };
    $scope.answer = function(answer) {
	$mdDialog.hide(answer);
    };
}
