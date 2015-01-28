angular.module("app").controller("OptionsSheetController",
				 function($scope, $mdBottomSheet, $mdDialog) {
    $scope.showControlsLayout = function(ev){
	$mdBottomSheet.hide(ev);
	$mdDialog.show({
            targetEvent: ev,
	    templateUrl: "/dialogs/controls_dialog.tpl.html",
	    controller: "ControlsDialogController"
        });
    };
    $scope.showDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: "/dialogs/diagnostics_dialog.tpl.html",
	    controller: "DiagnosticsDialogController"
        });
    };
    $scope.showGamepadDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: "/dialogs/gamepad_dialog.tpl.html",
	    controller: "GamepadDialogController"
        });
    };
});
