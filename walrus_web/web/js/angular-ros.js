var rosModule = angular.module("ros", []);

rosModule
    .provider('roslib',  {
	url: 'ws://localhost:9090',
	setUrl: function(url) {
	    this.url = url;
	},
	$get: ['$rootScope', function($rootScope) {
	    var ros = new ROSLIB.Ros({
		url : this.url
	    });
	    ros.on('connection', function() {
		$rootScope.$apply(function(){
		    $rootScope.$broadcast('ros-connection');
		});
	    });

	    ros.on('error', function(error) {
		$rootScope.$apply(function(){
		    $rootScope.$broadcast('ros-error', error);
		});
	    });

	    ros.on('close', function() {
		$rootScope.$apply(function(){
		    $rootScope.$broadcast('ros-close');
		});
	    });
	    return ros;
	}]
    });
