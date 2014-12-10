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
	    return {
		advertise: function(name, messageType) {
		    var topic = new ROSLIB.Topic({
			ros : ros,
			name : name,
			messageType : messageType
		    });
		    return {
			publish: function(msg){
			    topic.publish(new ROSLIB.Message(msg));
			}
		    };
		}
	    };
	}]
    })
    .provider('webrtcRosService',  {
	signalingUrl: 'ws://localhost:8080/webrtc',
	setSignalingUrl: function(signalingUrl) {
	    this.signalingUrl = signalingUrl;
	},
	$get: ['$rootScope', function($rootScope) {
	    return {
		signalingUrl: this.signalingUrl,
		createConnection: function() {
		    return WebrtcRos.createConnection(this.signalingUrl);
		}
	    };
	}]
    })
    .directive('webrtcRosVideo', ['webrtcRosService', function(webrtcRosService) {
	function link(scope, element, attrs) {
	    var connection = webrtcRosService.createConnection();
	    function configure() {
		config = {
		    "subscribed_video_topic": scope.topic,
		};
		connection.configure(config);
	    };
	    connection.onSignalingOpen = function(){
		console.log("Signaling open");
		configure();
	    };
	    connection.onSignalingError = function(error){
		console.error("Error opening signaling channel", error);
	    };
	    connection.onRemoteStreamAdded = function(event) {
		console.log("Remote stream added:", URL.createObjectURL(event.stream));
		element[0].src = URL.createObjectURL(event.stream);
		element[0].play();
	    }
	    connection.onRemoteStreamRemoved = function(event) {
		console.log("Remote stream removed");
	    }
	}

	return {
	    scope: {
		topic: '=topic'
	    },
	    restrict: 'A',
	    link: link
	};
    }]);
