angular.module("ros").service("diagnosticsService", function($rootScope, roslib) {
    var diagnostics_callback = function(msg) {
	var diagnostics = {
	    msg: msg,
	    // Use the item name and value key to extract a value from the diagnostics message
	    // Optionally apply a regex to the value and extract the first match group
	    value: function(name, key, pattern) {
		var item = this.msg.status.filter(function(item){
		    return item.name === name;
		})[0];
		if(!item) {
		    return null;
		}
		var value = item.values.filter(function(v){
		    return v.key === key;
		})[0];
		if(!value) {
		    return null;
		}
		if(!pattern) {
		    return value.value;
		}
		var match_result = value.value.match(pattern);
		if(match_result) {
		    return match_result[1];
		}
		else {
		    return null;
		}
	    },
	    state: function(name) {
		var item = this.msg.status.filter(function(item){
		    return item.name === name;
		})[0];
		if(item) {
		    return item.level;
		}
		else {
		    return null;
		}
	    }
	};
	$rootScope.$broadcast("ros-diagnostics", diagnostics);
    };
    var diagnostics_sub = roslib.subscribe("/diagnostics_agg", "diagnostic_msgs/DiagnosticArray", diagnostics_callback);

    return {
	OK: 0,
	WARN: 1,
	ERROR: 2,
	STALE: 3
    };
});
