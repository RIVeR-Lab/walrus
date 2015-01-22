angular.module("app").controller("DiagnosticsDialogController",
				 function($scope, $mdDialog, roslib) {

  var OK = 0;
  var WARN = 1;
  var ERROR = 2;
  var STALE = 3;

  var diagnostics_callback = function(diagnostics) {
      $scope.diagnostics = diagnostics;
      console.log(diagnostics);
  };
  var diagnostics_sub = roslib.subscribe("/diagnostics_agg", "diagnostic_msgs/DiagnosticArray", diagnostics_callback);
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
      if(level === OK) {
	  return "green";
      }
      else if(level === WARN) {
	  return "orange";
      }
      else if(level === STALE) {
	  return "blue";
      }
      else {
	  return "red";
      }
  };
  $scope.statusLevelToSymbol = function(level) {
      if(level === OK) {
	  return "\u2714";
      }
      else if(level === WARN) {
	  return "\u2757";
      }
      else if(level === STALE) {
	  return "\u2753";
      }
      else {
	  return "\u2716";
      }
  };
});
