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
});
