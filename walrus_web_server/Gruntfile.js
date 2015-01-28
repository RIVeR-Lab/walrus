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
		maxparams: 4,
		maxdepth: 2,
		maxstatements: 16,
		maxcomplexity: 5
	    },
	    files: ["Gruntfile.js", "src/**/*.js"]
	},
	csslint: {
	    strict: {
		src: ["src/**/*.css"]
	    }
	},
	cssmin: {
	    target: {
		files: {
		    "web/<%= pkg.name %>.min.css": ["src/**/*.css"]
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
		    htmlmin:  "<%= htmlmin.app.options %>"
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
	    js: {
		src: [
		    // make sure that the modules definitions are first
		    "build/annotated/app.annotated.js",
		    "build/annotated/services/ros/ros.annotated.js",
		    "build/annotated/**/*.js",
		    "<%= ngtemplates.html_templates.dest %>",
		    "<%= ngtemplates.svg_templates.dest %>"
		],
		dest: "build/<%= pkg.name %>.js"
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
	    js: {
		files: {
		    "web/<%= pkg.name %>.min.js": ["<%= concat.js.dest %>"]
		}
	    }
	},
	htmlmin: {
	    app: {
		options: {
		    removeComments: true,
		    collapseWhitespace: true
		},
		files: [{
		    expand: true,
		    cwd: "src/",
		    src: "index.html",
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
		    "angular-animate/angular-animate.min.js", "angular-animate/angular-animate.min.js.map",
		    "angular-aria/angular-aria.min.js", "angular-aria/angular-aria.min.js.map",
		    "angular-material/angular-material.min.js", "angular-material/angular-material.min.css",
		    "eventemitter2/lib/eventemitter2.js",
		    "roslib/build/roslib.min.js",
		    "hammerjs/hammer.min.js", "hammerjs/hammer.min.map"
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
