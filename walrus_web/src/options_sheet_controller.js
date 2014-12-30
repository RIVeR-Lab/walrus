angular.module("app").controller("OptionsSheetController",
					  function($scope, $mdBottomSheet, $mdDialog, diagnostics) {
    $scope.showControlsLayout = function(ev){
	$mdBottomSheet.hide(ev);
	$mdDialog.show({
            targetEvent: ev,
	    templateUrl: "dialogs/controls_dialog.html",
	    controller: "ControlsDialogController"
        });
    };
    $scope.showDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: "dialogs/diagnostics_dialog.html",
	    locals: { diagnostics: diagnostics },
	    controller: "DiagnosticsDialogController"
        });
    };
    $scope.showGamepadDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: "dialogs/gamepad_dialog.html",
	    controller: "GamepadDialogController"
        });
    };
});
