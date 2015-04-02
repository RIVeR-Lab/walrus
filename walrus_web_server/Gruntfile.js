var build_tasks = ["jshint", "csslint", "cssmin", "ngAnnotate", "ngtemplates", "concat", "uglify", "copy:images", "copy:bower_libs", "copy:libs", "htmlmin"];
module.exports = function(grunt) {
    "use strict";
    grunt.initConfig({
	pkg: grunt.file.readJSON("package.json"),
	jshint: {
	    options: {
		browser: true,
		devel: true,
		globals: {
		    angular: false,
		    ROSLIB: false,
		    ROS3D: false,
		    module: false,
		    RTCPeerConnection: true,
		    RTCSessionDescription: true,
		    RTCIceCandidate: true,
		    URL: true
		},

		enforceall: true,
		camelcase: false,
		nocomma: false,
		strict: false,
		singleGroups: false,

		quotmark: "double",
		maxparams: 6,
		maxdepth: 3,
		maxstatements: 30,
		maxcomplexity: 6
	    },
	    files: ["Gruntfile.js", "src/**/*.js"]
	},
	csslint: {
	    strict: {
		src: ["src/**/*.css"]
	    }
	},
	cssmin: {
	    web_interface: {
		files: {
		    "web/web_interface.min.css": ["src/web_interface/**/*.css"]
		}
	    },
	    tools: {
		files: {
		    "web/tools/app.min.css": ["src/tools/**/*.css"]
		}
	    }
	},
	ngAnnotate: {
            options: {
		singleQuotes: false
            },
            app: {
                files: [{
                    expand: true,
		    cwd: "src",
                    src: ["**/*.js"],
                    ext: ".annotated.js",
                    extDot: "last",
		    dest: "build/annotated/"
                }]
            }
	},
	ngtemplates: {
	    html_templates: {
		options: {
		    prefix: "/",
		    htmlmin:  "<%= htmlmin.options %>"
		},
		cwd: "src",
		src: "**/*.tpl.html",
		dest: "build/html-templates.js"
	    },
	    svg_templates: {
		options: {
		    prefix: "/"
		},
		cwd: "src",
		src: "**/*.tpl.svg",
		dest: "build/svg-templates.js"
	    }
	},
	concat: {
	    options: {
		separator: ";"
	    },
	    web_interface: {
		src: [
		    // make sure that the modules definitions are first
		    "build/annotated/web_interface/app.annotated.js",
		    "build/annotated/services/ros/ros.annotated.js",
		    "build/annotated/services/gamepad_service.annotated.js",
		    "build/annotated/services/**/*.js",
		    "build/annotated/directives/**/*.js",
		    "build/annotated/util/**/*.js",
		    "build/annotated/web_interface/**/*.js",
		    "<%= ngtemplates.html_templates.dest %>",
		    "<%= ngtemplates.svg_templates.dest %>"
		],
		dest: "build/web_interface.js"
	    },
	    tools: {
		src: [
		    // make sure that the modules definitions are first
		    "build/annotated/tools/app.annotated.js",
		    "build/annotated/services/ros/ros.annotated.js",
		    "build/annotated/services/gamepad_service.annotated.js",
		    "build/annotated/services/**/*.js",
		    "build/annotated/directives/**/*.js",
		    "build/annotated/util/**/*.js",
		    "build/annotated/tools/**/*.js",
		    "<%= ngtemplates.html_templates.dest %>",
		    "<%= ngtemplates.svg_templates.dest %>"
		],
		dest: "build/tools/app.js"
	    }
	},
	uglify: {
	    options: {
		banner: "/*! <%= pkg.name %> <%= grunt.template.today('dd-mm-yyyy') %> */\n" +
		    ";(function( window, undefined ){ \n 'use strict';",
		footer: "\n}( window ));",
		sourceMap: true,
		sourceMapIncludeSources: true,
		mangle: true
	    },
	    web_interface: {
		files: {
		    "web/web_interface.min.js": ["<%= concat.web_interface.dest %>"]
		}
	    },
	    tools: {
		files: {
		    "web/tools/app.min.js": ["<%= concat.tools.dest %>"]
		}
	    }
	},
	htmlmin: {
	    options: {
		removeComments: true,
		collapseWhitespace: true
	    },
	    web_interface: {
		files: [{
		    expand: true,
		    cwd: "src/web_interface",
		    src: "index.html",
		    dest: "web/"
		}]
	    },
	    tools: {
		files: [{
		    expand: true,
		    cwd: "src",
		    src: "tools/index.html",
		    dest: "web/"
		}]
	    }
	},
	copy: {
	    images: {
		nonull: true,
		expand: true,
		cwd: "src/",
		src: ["img/**"],
		dest: "web/"
	    },
	    libs: {
		nonull: true,
		expand: true,
		cwd: "libs",
		src: ["**"],
		dest: "web/libs/"
	    },
	    bower_libs: {
		nonull: true,
		expand: true,
		cwd: "bower_components",
		src: [
		    "angular/angular.min.js", "angular/angular.min.js.map",
		    "angular-route/angular-route.min.js", "angular-route/angular-route.min.js.map",
		    "angular-animate/angular-animate.min.js", "angular-animate/angular-animate.min.js.map",
		    "angular-aria/angular-aria.min.js", "angular-aria/angular-aria.min.js.map",
		    "angular-material/angular-material.min.js", "angular-material/angular-material.min.css"
		],
		dest: "web/libs/"
	    }
	},
	watch: {
	    options: {
		atBegin: true
	    },
	    files: ["src/**", "Gruntfile.js"],
	    tasks: build_tasks
	},
	clean: ["web/", "build/"]
    });

    grunt.loadNpmTasks("grunt-contrib-jshint");
    grunt.loadNpmTasks("grunt-ng-annotate");
    grunt.loadNpmTasks("grunt-angular-templates");
    grunt.loadNpmTasks("grunt-contrib-csslint");
    grunt.loadNpmTasks("grunt-contrib-cssmin");
    grunt.loadNpmTasks("grunt-contrib-htmlmin");
    grunt.loadNpmTasks("grunt-contrib-concat");
    grunt.loadNpmTasks("grunt-contrib-uglify");
    grunt.loadNpmTasks("grunt-contrib-htmlmin");
    grunt.loadNpmTasks("grunt-contrib-copy");

    grunt.loadNpmTasks("grunt-contrib-watch");
    grunt.loadNpmTasks("grunt-contrib-clean");

    grunt.registerTask("default", build_tasks);
};
