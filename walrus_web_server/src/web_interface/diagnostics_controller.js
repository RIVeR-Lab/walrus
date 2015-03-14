angular.module("app").controller("DiagnosticsCtrl",
				 function( $scope, $mdDialog, $mdBottomSheet, diagnosticsService, gamepadService ) {
    $scope.ros = {
	connected: false
    };

    $scope.gamepad = gamepadService.gamepad;

    $scope.$on("ros-connection", function() {
	$scope.ros.connected = true;
    });
    $scope.$on("ros-error", function(/*error*/) {
	$scope.ros.connected = false;
    });
    $scope.$on("ros-close", function() {
	$scope.ros.connected = false;
    });

    $scope.$on("ros-diagnostics", function(e, diagnostics) {
	var percent_regex = /^(\d+(?:\.\d+)?)%$/; // matches ##.##%

	var cpu_percent = diagnostics.value("/Computers/Primary Computer/Resources/CPU", "Total CPU", percent_regex);
	$scope.diagnostics.cpu.value = cpu_percent / 100;
	$scope.diagnostics.cpu.state = diagnostics.state("/Computers/Primary Computer/Resources/CPU");

	var memory_percent = diagnostics.value("/Computers/Primary Computer/Resources/Virtual Memory", "Percent Used", percent_regex);
	$scope.diagnostics.memory.value = memory_percent / 100;
	$scope.diagnostics.memory.state = diagnostics.state("/Computers/Primary Computer/Resources/Virtual Memory");

	var temp_regex = /^(\d+(?:\.\d+)?)/; // matches ##.##
	var cpu_temp = diagnostics.value("/Computers/Primary Computer/Sensors/coretemp-isa-0000 Physical id 0", "Temperature", temp_regex);
	$scope.diagnostics.cpu_temp.value = cpu_temp;
	$scope.diagnostics.cpu_temp.state = diagnostics.state("/Computers/Primary Computer/Sensors/coretemp-isa-0000 Physical id 0");
    });


    $scope.diagnostics = {
	cpu: {value: 1.0, state: diagnosticsService.STALE},
	memory: {value: 1.0, state: diagnosticsService.STALE},
	cpu_temp: {value: 0.0, state: diagnosticsService.STALE},
	batteries: [
	    {value: 0.0, connected: false, state: diagnosticsService.ERROR},
	    {value: 0.2, connected: true, state: diagnosticsService.WARN},
	    {value: 0.5, connected: true, state: diagnosticsService.OK},
	    {value: 0.8, connected: true, state: diagnosticsService.STALE}
	],
	drive: [
	    {value: 0.1, state: diagnosticsService.STALE},
	    {value: -0.2, state: diagnosticsService.STALE}
	]
    };


    $scope.stateToColor = function(state){
	if(state === diagnosticsService.ERROR) {
	    return "red";
	}
	else if(state === diagnosticsService.WARN) {
	    return "orange";
	}
	else if(state === diagnosticsService.OK) {
	    return "green";
	}
	else {
	    return "blue";
	}
    };

    $scope.Math = window.Math;

    $scope.showOptions = function(ev){
	$scope.alert = "";
	$mdBottomSheet.show({
	    templateUrl: "/web_interface/options_sheet.tpl.html",
	    controller: "OptionsSheetController",
	    targetEvent: ev
	});
    };

});
