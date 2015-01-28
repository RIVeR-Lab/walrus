angular.module("ros").provider("webrtcRosService",  {
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
.directive("webrtcRosVideo", function(webrtcRosService) {
        var DISCONNECTED_STATE = "disconnected";
        var CONNECTED_STATE = "connected";
        var STREAMING_STATE = "streaming";
	function link(scope, element/*, attrs*/) {
	    var connection = webrtcRosService.createConnection();
	    var signaling_open = false;
	    var last_topic = "";

	    function fireStateChanged(state) {
		scope.onStateChange({state: state, console: console});
	    }

	    function configure() {
		if(scope.topic !== last_topic) {
		    console.log("Video topic change: " + last_topic + " -> " + scope.topic);
		    var config = {
			"subscribed_video_topic": scope.topic
		    };
		    connection.configure(config);
		    last_topic = scope.topic;
		}
	    }
	    connection.onSignalingOpen = function(event){
		fireStateChanged(CONNECTED_STATE);
		console.log("Signaling open", event);
		signaling_open = true;
		configure();
	    };
	    connection.onSignalingError = function(error){
		console.log("Signaling channel error", error);
		fireStateChanged(DISCONNECTED_STATE);
		signaling_open = false;
	    };
	    connection.onSignalingClose = function(event){
		console.log("Signaling channel closed", event);
		fireStateChanged(DISCONNECTED_STATE);
		signaling_open = false;
	    };
	    connection.onRemoteStreamAdded = function(event) {
		console.log("Remote stream added:", URL.createObjectURL(event.stream));
		element[0].src = URL.createObjectURL(event.stream);
		fireStateChanged(STREAMING_STATE);
	    };
	    connection.onRemoteStreamRemoved = function(/*event*/) {
		fireStateChanged(CONNECTED_STATE);
		console.log("Remote stream removed");
	    };
	    scope.$watch("topic", function(newValue, oldValue) {
		if(signaling_open) {
		    configure();
		}
	    });
	    fireStateChanged(DISCONNECTED_STATE);
	}

	return {
	    scope: {
		topic: "=topic",
		onStateChange: "&onStateChange"
	    },
	    restrict: "A",
	    link: link
	};
});
