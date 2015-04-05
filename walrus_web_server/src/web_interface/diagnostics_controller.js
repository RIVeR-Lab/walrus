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

	for(var i = 0; i < 4; ++i) {
	    var battery_path = "/Power/Battery " + (i + 1);
	    $scope.diagnostics.batteries[i].connected = diagnostics.value(battery_path, "Present") === "Yes";
	    $scope.diagnostics.batteries[i].state = diagnostics.state(battery_path);
	    var charge_percent = diagnostics.value(battery_path, "Charge", percent_regex);
	    $scope.diagnostics.batteries[i].value = charge_percent / 100;
	}

	var current_regex = /^(-?\d+(?:\.\d+)?)\s*A$/; // matches ##.## A

	var drive_motor_names = ["left", "right"];
	var drive_motor_directions = [1, -1];
	for(var j = 0; j < drive_motor_names.length; ++j) {
	    var motor_name = drive_motor_names[j];
	    var path = "/Drive/"+(motor_name.charAt(0).toUpperCase() + motor_name.slice(1))+" Drive";
	    var output_path = path+"/base_epos: "+motor_name+"_drive_actuator: Motor Output";
	    var output_current = diagnostics.value(output_path, "Current", current_regex);
	    var nominal_current = diagnostics.value(output_path, "Nominal Current", current_regex);
	    var max_current = diagnostics.value(output_path, "Max Current", current_regex);
	    $scope.diagnostics.drive[j].state = diagnostics.state(path);
	    $scope.diagnostics.drive[j].current_limit_active = diagnostics.value(output_path, "Current Limit Active") === "True";
	    $scope.diagnostics.drive[j].value = drive_motor_directions[j] * output_current / max_current;
	}
    });


    $scope.diagnostics = {
	cpu: {value: 1.0, state: diagnosticsService.STALE},
	memory: {value: 1.0, state: diagnosticsService.STALE},
	cpu_temp: {value: 0.0, state: diagnosticsService.STALE},
	batteries: [
	    {value: 0.0, connected: false, state: diagnosticsService.STALE},
	    {value: 0.0, connected: false, state: diagnosticsService.STALE},
	    {value: 0.0, connected: false, state: diagnosticsService.STALE},
	    {value: 0.0, connected: false, state: diagnosticsService.STALE}
	],
	drive: [
	    {value: 0.0, state: diagnosticsService.STALE, current_limit_active: false},
	    {value: 0.0, state: diagnosticsService.STALE, current_limit_active: false}
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
