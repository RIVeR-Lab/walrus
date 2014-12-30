angular.module("app").directive("videoViewer", function() {
  return {
      scope: {
	  topic: "=topic",
	  label: "=label"
      },
    templateUrl: "/directives/video_viewer.tpl.html"
  };
});
