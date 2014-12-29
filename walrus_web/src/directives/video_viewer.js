angular.module("app").directive("videoViewer", function() {
  return {
      scope: {
	  topic: "=topic",
	  label: "=label",
	  insetShadow: "=insetshadow"
      },
    templateUrl: "directives/video_viewer.template.html"
  };
});
