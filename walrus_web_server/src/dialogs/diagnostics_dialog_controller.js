angular.module("app").controller("DiagnosticsDialogController",
				 function($scope, $mdDialog, roslib, diagnosticsService) {

  $scope.$on("ros-diagnostics", function(e, diagnostics) {
      $scope.diagnostics = diagnostics.msg;
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
  $scope.statusLevelToColor = function(level) {
      if(level === diagnosticsService.OK) {
	  return "green";
      }
      else if(level === diagnosticsService.WARN) {
	  return "orange";
      }
      else if(level === diagnosticsService.STALE) {
	  return "blue";
      }
      else {
	  return "red";
      }
  };
  $scope.statusLevelToSymbol = function(level) {
      if(level === diagnosticsService.OK) {
	  return "\u2714";
      }
      else if(level === diagnosticsService.WARN) {
	  return "\u2757";
      }
      else if(level === diagnosticsService.STALE) {
	  return "\u2753";
      }
      else {
	  return "\u2716";
      }
  };
});
