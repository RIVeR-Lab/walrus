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
	function link(scope, element/*, attrs*/) {
	    var connection = webrtcRosService.createConnection();
	    var signaling_open = false;
	    var last_topic = "";
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
	    connection.onSignalingOpen = function(){
		console.log("Signaling open");
		signaling_open = true;
		configure();
	    };
	    connection.onSignalingError = function(error){
		console.error("Error opening signaling channel", error);
		signaling_open = false;
	    };
	    connection.onRemoteStreamAdded = function(event) {
		console.log("Remote stream added:", URL.createObjectURL(event.stream));
		element[0].src = URL.createObjectURL(event.stream);
		element[0].play();
	    };
	    connection.onRemoteStreamRemoved = function(/*event*/) {
		console.log("Remote stream removed");
	    };
	    scope.$watch("topic", function(newValue, oldValue) {
		if(signaling_open) {
		    configure();
		}
	    });
	}

	return {
	    scope: {
		topic: "=topic"
	    },
	    restrict: "A",
	    link: link
	};
});
