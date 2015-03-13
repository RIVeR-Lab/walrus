angular.module("app").controller("RootCtrl",
				 function( $scope, $mdSidenav, $location ) {
    $scope.$location = $location;
    $scope.toolGroups = [
	{ name: "WALRUS",
	  tools: [
	      {name: "Pod Control"}
	  ]
	},
	{ name: "ROS",
	  tools: [
	      {name: "Message Tool", path: "#ros-message-tool"},
	      {name: "ROS Log", path: "#ros-log-tool"}
	  ]
	}
    ];

    $scope.toggleSidenav = function() {
	$mdSidenav("left").toggle();
    };
});
