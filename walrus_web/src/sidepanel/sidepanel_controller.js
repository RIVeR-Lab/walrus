angular.module("app").controller("SidepanelCtrl", ["$scope", function( $scope ) {
    $scope.tabs = {
	selectedIndex : 0
    };
    $scope.$watch("tabs.selectedIndex", function(newIndex, oldIndex){
	if(newIndex !== oldIndex) {
	    setTimeout(function() { // Need to delay to allow for the tabs to actually change
		$scope.$broadcast("tab-changed", newIndex, oldIndex);
	    }, 100);
	}
    });
}]);
