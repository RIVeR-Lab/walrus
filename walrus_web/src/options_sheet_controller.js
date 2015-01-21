angular.module("app").controller("OptionsSheetController",
				 function($scope, $mdBottomSheet, $mdDialog, diagnostics) {
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
    $scope.showFakeDiagnostics = function(ev){
	$mdBottomSheet.hide(ev);
        $mdDialog.show({
            targetEvent: ev,
	    templateUrl: "/dialogs/fake_diagnostics_dialog.tpl.html",
	    locals: { diagnostics: diagnostics },
	    controller: "FakeDiagnosticsDialogController"
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
