angular.module("ros", [])
    .provider("roslib",  {
	rosbridgeWsUrl: "ws://localhost:9090/",
	packageUrl: "http://localhost:8080/",
	setRosbridgeWsUrl: function(url) {
	    this.rosbridgeWsUrl = url;
	},
	setPackageUrl: function(url) {
	    this.packageUrl = url;
	},
	$get: ["$rootScope", function($rootScope) {
	    var ros = new ROSLIB.Ros({
		url : this.rosbridgeWsUrl
	    });
	    ros.on("connection", function() {
		$rootScope.$apply(function(){
		    $rootScope.$broadcast("ros-connection");
		});
	    });

	    ros.on("error", function(error) {
		$rootScope.$apply(function(){
		    $rootScope.$broadcast("ros-error", error);
		});
	    });

	    ros.on("close", function() {
		$rootScope.$apply(function(){
		    $rootScope.$broadcast("ros-close");
		});
	    });
	    return {
		ros: ros,
		packageUrl: this.packageUrl,
		createTfClient: function(fixedFrame) {
		    return new ROSLIB.TFClient({
			ros : this.ros,
			angularThres : 0.01,
			transThres : 0.01,
			rate : 10.0,
			fixedFrame: fixedFrame
		    });
		},
		createUrdfClient: function(scene, fixedFrame) {
		    return new ROS3D.UrdfClient({
			ros : this.ros,
			tfClient : this.createTfClient(fixedFrame),
			path : this.packageUrl,
			rootObject : scene
		    });
		},
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
    .directive("urdfViewer", ["roslib", function(roslib) {
	function link(scope, element/*, attrs*/) {
	    if(!element.attr("id")) {
		throw "A urdf viewer must define an id";
	    }
	    var viewer = new ROS3D.Viewer({
		divID : element.attr("id"),
		width : element[0].offsetWidth,
		height : element[0].offsetWidth / scope.aspectRatio,
		background: "#AAAAAA",
		antialias : true
	    });
	    viewer.addObject(new ROS3D.Grid());
	    roslib.createUrdfClient(viewer.scene, scope.fixedFrame);

	    function updateViewer() {
		// only resize if the viewer is actually visible
		if(element[0].offsetParent !== null) {
		    var newWidth = element[0].offsetWidth;
		    var newHeight = element[0].offsetWidth / scope.aspectRatio;
		    viewer.renderer.setSize(newWidth, newHeight);
		    viewer.camera.updateProjectionMatrix();
		}
	    }

	    window.addEventListener("resize", updateViewer);
	    scope.$on("tab-changed", function(/*ev, newIndex, oldIndex*/) {
		updateViewer();
	    });
	}

	return {
	    scope: {
		"aspectRatio": "=aspectRatio",
		"fixedFrame": "=fixedFrame"
	    },
	    restrict: "E",
	    link: link
	};
    }])

    .provider("webrtcRosService",  {
	signalingUrl: "ws://localhost:8080/webrtc",
	setSignalingUrl: function(signalingUrl) {
	    this.signalingUrl = signalingUrl;
	},
	$get: [function() {
	    return {
		signalingUrl: this.signalingUrl,
		createConnection: function() {
		    return window.WebrtcRos.createConnection(this.signalingUrl);
		}
	    };
	}]
    })
    .directive("webrtcRosVideo", ["webrtcRosService", function(webrtcRosService) {
	function link(scope, element/*, attrs*/) {
	    var connection = webrtcRosService.createConnection();
	    function configure() {
		var config = {
		    "subscribed_video_topic": scope.topic
		};
		connection.configure(config);
	    }
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
	    };
	    connection.onRemoteStreamRemoved = function(/*event*/) {
		console.log("Remote stream removed");
	    };
	}

	return {
	    scope: {
		topic: "=topic"
	    },
	    restrict: "A",
	    link: link
	};
    }]);
