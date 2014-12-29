angular.module("app").controller("DiagnosticsDialogController", ["$scope", "$mdDialog", "diagnostics",
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
