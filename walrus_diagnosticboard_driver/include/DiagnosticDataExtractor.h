#ifndef DIAGNOSTICDATAEXTRACTOR_H
#define DIAGNOSTICDATAEXTRACTOR_H

#include <string>
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "boost/function.h"
#include <vector>

using namespace diagnostic_msgs;

class DiagnosticDataExtractor
{
private:
    boost::function<void ()> onNewData;
    NodeHandle nh;
    DiagnosticArray diagnostic_data;
    DiagnosticStatus cached_status;
    
    void aggregator_callback(const DiagnosticArray &msg)
    {
        diagnostic_data = msg;
        onNewData();
    }
    
    bool cache_status(string label)
    {
        if (cached_status.name == label)
            return true;
    }
    
public:
    const TYPE_STATUS = 0;
    const TYPE_GROUP = 1;
    const TYPE_INVALID = 2;
    
    DiagnosticDataExtractor(NodeHandle &nh)
    : nh(nh), cached_status()
    {
        cached_status.name = "INVALID";
    }
    
    DiagnosticDataExtractor(NodeHandle &nh, boost::function<void ()> &onNewData)
    : nh(nh), onNewData(onNewData), cached_status()
    {
    }
    
    int getType(string label)
    {
        if (cache_status(label))
            return TYPE_STATUS = 0;
    }
    
    DiagnosticStatus getStatus(string label)
    {       
        
    }
    
    vector<string> getGroupMembers(string label)
    {
        
    }

    int getLevel(string label)
    {
        if (!cache_status(label))
            return DiagnosticStatus::STALE;
        return cached_status.level;
    }
    
    string getMessage(string label)
    {
        if (!cache_status(label))
            return "INVALID LABEL";
        return cached_status.message;
    }
    
    string getHardwareID(string label)
    {
        if (!cache_status(label))
            return "INVALID LABEL";
        return cached_status.hardware_id;
    }
    
    vector<KeyValue> getKeyValuePairs(string label)
    {
        if (!cache_status(label))
            return "INVALID LABEL";
        return cached_status.values;
    }
    
    vector<string> getKeys(string label)
    {
        if (!cache_status(label))
            return "INVALID LABEL";
        vector<string> keys;
        for (int l = 0; l < cached_status.values.size(); l++)
            keys.push_back(cached_status.values[l].key);
        return keys;        
    }
    
    string getValue(string label, string key)
    {
        if (!cache_status(label))
            return "INVALID LABEL";
        for (int l = 0; l < cached_status.values.size(); l++)
        {
            if (cached_status.values[l].key == key)
                return cached_status.values[l].value;
        }
        return "INVALID KEY";
    }
    
    int getValueAsInt(string label, string key)
    {
       string value = getValue(label, key);
       
    }
    
    double getValueAsDouble(string label, string key)
    {
        string value = getValue(label, key);
        
    }
    
    bool getValueAsBool(string label, string key)
    {
        string value = getValue(label, key);
        
    }
    
};

#endif
