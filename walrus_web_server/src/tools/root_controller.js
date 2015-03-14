angular.module("app").controller("RootCtrl",
				 function( $scope, $mdSidenav, $location, toolGroups ) {
    $scope.$location = $location;
    $scope.toolGroups = toolGroups;

    $scope.toggleSidenav = function() {
	$mdSidenav("left").toggle();
    };
});
