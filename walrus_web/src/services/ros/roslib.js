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
	$get: function($rootScope) {
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
		createUrdfClient: function(scene, fixedFrame, param) {
		    return new ROS3D.UrdfClient({
			ros : this.ros,
			tfClient : this.createTfClient(fixedFrame),
			path : this.packageUrl,
			rootObject : scene,
			param: param
		    });
		},
		advertise: function(name, messageType) {
		    var topic = new ROSLIB.Topic({
			ros : ros,
			name : name,
			messageType : messageType
		    });
		    return {
			shutdown: function(){
			    topic.unadvertise();
			},
			publish: function(msg){
			    topic.publish(new ROSLIB.Message(msg));
			}
		    };
		},
		subscribe: function(name, messageType, callback) {
		    var topic = new ROSLIB.Topic({
			ros : ros,
			name : name,
			messageType : messageType
		    });
		    topic.subscribe(callback);
		    return {
			shutdown: function(){
			    topic.unsubscribe();
			}
		    };
		}
	    };
	}
    });
