var app = angular.module('app', ['ros', 'gamepad', 'ngMaterial']);

app.config(["roslibProvider", function(roslibProvider){
    roslibProvider.setUrl('ws://localhost:9003');
}]);

app.controller('RootCtrl', function( $scope, roslib, gamepadService ) {
});

app.controller('SidepanelCtrl', function( $scope ) {
    $scope.tabs = {
	selectedIndex : 0
    };
});

app.controller('DiagnosticsCtrl', function( $scope, $mdDialog ) {
    $scope.ros = {
	connected: false
    };

    $scope.gamepad = {
	connected: false,
    };

    $scope.$on('gamepad-connected', function() {
	$scope.gamepad.connected = true;
    });
    $scope.$on('gamepad-disconnected', function() {
	$scope.gamepad.connected = false;
    });

    $scope.$on('ros-connection', function() {
	$scope.ros.connected = true;
    });
    $scope.$on('ros-error', function(error) {
	$scope.ros.connected = false;
    });
    $scope.$on('ros-close', function() {
	$scope.ros.connected = false;
    });

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


function GamepadDialogController($scope, $mdDialog, $interval, gamepadService) {
    $scope.gamepad = {
	connected: gamepadService.isConnected(),
	data: gamepadService.getLastData()
    };

    $scope.$on('gamepad-connected', function() {
	$scope.gamepad.connected = true;
    });
    $scope.$on('gamepad-disconnected', function() {
	$scope.gamepad.connected = false;
    });
    $scope.$on('gamepad-data', function(ev, data) {
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
}
