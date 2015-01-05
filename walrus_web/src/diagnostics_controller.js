angular.module("app").controller("DiagnosticsCtrl",
				 function( $scope, $mdDialog, $mdBottomSheet ) {
    $scope.ros = {
	connected: false
    };

    $scope.gamepad = {
	connected: false
    };

    $scope.$on("gamepad-connected", function() {
	$scope.gamepad.connected = true;
    });
    $scope.$on("gamepad-disconnected", function() {
	$scope.gamepad.connected = false;
    });

    $scope.$on("ros-connection", function() {
	$scope.ros.connected = true;
    });
    $scope.$on("ros-error", function(/*error*/) {
	$scope.ros.connected = false;
    });
    $scope.$on("ros-close", function() {
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
	if(val > 0.8) {
	    return "red";
	}
	else if(val > 0.6) {
	    return "orange";
	}
	else {
	    return "green";
	}
    };
    $scope.percentToColorLowBad = function(val){
	if(val < 0.2) {
	    return "red";
	}
	else if(val < 0.4) {
	    return "orange";
	}
	else {
	    return "green";
	}
    };

    $scope.Math = window.Math;

    $scope.showOptions = function(ev){
	$scope.alert = "";
	$mdBottomSheet.show({
	    templateUrl: "/options_sheet.tpl.html",
	    controller: "OptionsSheetController",
	    targetEvent: ev,
	    locals: { diagnostics: $scope.diagnostics }
	});
    };

});
