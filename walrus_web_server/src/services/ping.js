var pingModule = angular.module("ping", []);

pingModule
    .provider("pingService",  {
	$get: function($rootScope, $interval, $q) {
	    var ping = function(ip, path, failOnError, bustCache) {
		path = path || "/";
		bustCache = bustCache || true;
		failOnError = failOnError || true;

		var deferred = $q.defer();
		var img = new Image();
		var start = new Date();
		var complete = function() {
		    var end = new Date();
		    deferred.resolve(end.getTime()-start.getTime());
		};
		img.onload = function(e) {
		    complete();
		};
		img.onerror = function(e) {
		    if(failOnError) {
			deferred.reject();
		    } else {
			complete();
		    }
		};
		img.src = "http://"+ip+path+(bustCache?"?v="+new Date().getTime():"");
		$interval(function() {
		    deferred.reject();
		}, /* delay */ 2000, /* count */ 1);
		return deferred.promise;
	    };
	    var repeatPing = function(callback, delay, ip, path, failOnError, bustCache) {
		return $interval(function() {
		    ping(ip, path, failOnError, bustCache).then(function(time) {
			callback(true, time);
		    }, function() {
			callback(false);
		    });
		}, delay);
	    };
	    return {
		ping: ping,
		repeatPing: repeatPing
	    };
	}
    });
