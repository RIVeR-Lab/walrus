var app = angular.module('app', ['ros', 'gamepad', 'ngMaterial']);

app.config(["roslibProvider", "gamepadServiceProvider", "webrtcRosServiceProvider",
	    function(roslibProvider, gamepadServiceProvider, webrtcRosServiceProvider){
    roslibProvider.setRosbridgeWsUrl('ws://'+location.hostname+':9003');
    roslibProvider.setPackageUrl('http://'+location.hostname+':9002/');

    webrtcRosServiceProvider.setSignalingUrl('ws://'+location.hostname+':9001/webrtc');

    gamepadServiceProvider.setPollRate(100);
}]);


app.controller('RootCtrl', ['$scope', 'roslib', 'gamepadService', 'webrtcRosService',
			    function( $scope, roslib, gamepadService, webrtcRosService ) {
    var joyPub = roslib.advertise('/joy', 'sensor_msgs/Joy');
    $scope.$on('gamepad-data', function(ev, data) {
	var currentTime = new Date();
	var secs = Math.floor(currentTime.getTime()/1000);
        var nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
	var joyData = {
	    header: { stamp: { secs: secs, nsecs: nsecs } },
	    axes: data.axes,
	    buttons: data.buttons.map(function(button){return button.pressed?1:0;})};
	joyData.axes[0] = -joyData.axes[0];
	joyData.axes[1] = -joyData.axes[1];
	joyPub.publish(joyData);
    });
    $scope.settings = {
	'touch_joystick' : true,
	'fake_video' : false
    };
}]);

app.directive('videoViewer', function() {
  return {
      scope: {
	  topic: '=topic',
	  label: '=label',
	  insetShadow: '=insetshadow'
      },
    templateUrl: 'video-viewer.template.html'
  };
});

app.controller('SidepanelCtrl', ['$scope', 'roslib', function( $scope, roslib ) {
    $scope.tabs = {
	selectedIndex : 0
    };
    $scope.$watch('tabs.selectedIndex', function(newIndex, oldIndex){
	if(newIndex !== oldIndex) {
	    setTimeout(function() { // Need to delay to allow for the tabs to actually change
		$scope.$broadcast('tab-changed', newIndex, oldIndex);
	    }, 100);
	}
    });
}]);

app.controller('DiagnosticsCtrl', ['$scope', '$mdDialog', '$mdBottomSheet',
				   function( $scope, $mdDialog, $mdBottomSheet ) {
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

    $scope.showOptions = function(ev){
	$scope.alert = '';
	$mdBottomSheet.show({
	    templateUrl: 'bottom_sheet.html',
	    controller: 'OptionsSheetController',
	    targetEvent: ev,
	    locals: { diagnostics: $scope.diagnostics },
	});
    };


}]);

app.controller('OptionsSheetController', ['$scope', '$mdBottomSheet', '$mdDialog', 'diagnostics',
					  function($scope, $mdBottomSheet, $mdDialog, diagnostics) {
    $scope.showControlsLayout = function(ev){
	$mdBottomSheet.hide(ev);
	$mdDialog.show({
            targetEvent: ev,
	    templateUrl: 'controls_dialog.html',
	    controller: 'ControlsDialogController'
        });
    };
    $scope.showDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: 'diagnostics_dialog.html',
	    locals: { diagnostics: diagnostics },
	    controller: 'DiagnosticsDialogController'
        });
    };
    $scope.showGamepadDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: 'gamepad_dialog.html',
	    controller: 'GamepadDialogController'
        });
    };
}]);

app.controller('DiagnosticsDialogController', ['$scope', '$mdDialog', 'diagnostics',
					       function($scope, $mdDialog, diagnostics) {
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
}]);


app.controller('ControlsDialogController', ['$scope', '$mdDialog',
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
}]);


app.controller('GamepadDialogController', ['$scope', '$mdDialog', 'gamepadService',
					   function($scope, $mdDialog, gamepadService) {
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
}]);
