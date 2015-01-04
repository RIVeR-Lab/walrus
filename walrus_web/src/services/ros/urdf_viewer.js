angular.module("ros").directive("urdfViewer", function(roslib) {
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
	    roslib.createUrdfClient(viewer.scene, scope.fixedFrame, scope.param);

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
		"fixedFrame": "=fixedFrame",
		"param": "=param"
	    },
	    restrict: "E",
	    link: link
	};
});
