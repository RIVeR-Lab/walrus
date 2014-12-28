build_tasks = ['jshint', 'csslint', 'cssmin', 'concat', 'uglify', 'copy:images', 'copy:bower_libs', 'copy:libs', 'htmlmin'];
module.exports = function(grunt) {
    grunt.initConfig({
	pkg: grunt.file.readJSON('package.json'),
	jshint: {
	    files: ['Gruntfile.js', 'src/**/*.js']
	},
	csslint: {
	    strict: {
		src: ['src/**/*.css']
	    }
	},
	cssmin: {
	    target: {
		files: {
		    'web/<%= pkg.name %>.min.css': ['src/**/*.css']
		}
	    }
	},
	concat: {
	    options: {
		separator: ';'
	    },
	    js: {
		src: ['src/**/*.js'],
		dest: 'build/<%= pkg.name %>.js'
	    }
	},
	uglify: {
	    options: {
		banner: '/*! <%= pkg.name %> <%= grunt.template.today("dd-mm-yyyy") %> */\n',
		sourceMap: true,
		//mangle: false
	    },
	    js: {
		files: {
		    'web/<%= pkg.name %>.min.js': ['<%= concat.js.dest %>']
		}
	    }
	},
	htmlmin: {
	    html: {
		options: {
		    removeComments: true,
		    collapseWhitespace: true
		},
		files: [{
		    expand: true,
		    cwd: 'src/',
		    src: '**/*.html',
		    dest: 'web/'
		}]
	    },
	},
	copy: {
	    images: {
		nonull: true,
		expand: true,
		cwd: 'src/',
		src: ['img/**', '**/*.svg'],
		dest: 'web/'
	    },
	    libs: {
		nonull: true,
		expand: true,
		cwd: 'libs',
		src: ['**'],
		dest: 'web/libs/'
	    },
	    bower_libs: {
		nonull: true,
		expand: true,
		cwd: 'bower_components',
		src: [
		    'angular/angular.min.js', 'angular/angular.min.js.map',
		    'angular-animate/angular-animate.min.js', 'angular-animate/angular-animate.min.js.map',
		    'angular-aria/angular-aria.min.js', 'angular-aria/angular-aria.min.js.map',
		    'angular-material/angular-material.min.js', 'angular-material/angular-material.min.css', 'angular-material/themes/*.css',
		    'eventemitter2/lib/eventemitter2.js',
		    'hammerjs/hammer.min.js', 'hammerjs/hammer.min.map'
		],
		dest: 'web/libs/'
	    }
	},
	watch: {
	    options: {
		atBegin: true
	    },
	    files: ['src/**', 'Gruntfile.js'],
	    tasks: build_tasks
	},
	clean: ['web/', 'build/']
    });

    grunt.loadNpmTasks('grunt-contrib-jshint');
    grunt.loadNpmTasks('grunt-contrib-csslint');
    grunt.loadNpmTasks('grunt-contrib-cssmin');
    grunt.loadNpmTasks('grunt-contrib-htmlmin');
    grunt.loadNpmTasks('grunt-contrib-concat');
    grunt.loadNpmTasks('grunt-contrib-uglify');
    grunt.loadNpmTasks('grunt-contrib-htmlmin');
    grunt.loadNpmTasks('grunt-contrib-copy');

    grunt.loadNpmTasks('grunt-contrib-watch');
    grunt.loadNpmTasks('grunt-contrib-clean');

    grunt.registerTask('default', build_tasks);
};
