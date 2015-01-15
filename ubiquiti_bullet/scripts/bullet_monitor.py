#!/usr/bin/env python

import roslib
roslib.load_manifest('ubiquiti_bullet')
import rospy
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import requests

def human_bytes(num_bytes):
    num_bytes = float(num_bytes)
    for x in ['B','KB','MB','GB']:
        if num_bytes < 1024:
            return "%3.1f %s" % (num_bytes, x)
        num_bytes /= 1024
    return "%3.1f%s" % (num_bytes, 'TB')

def human_wireless_mode(mode):
    if mode == 'ap':
        return 'Access Point'
    elif mode == 'sta':
        return 'Station'
    else:
        return mode

def update_callback(event):
    global ip_address
    global diagnostic_pub
    global device_name

    session = requests.Session()
    headers = {'Content-Type' : 'multipart/form-data; boundary=----WebKitFormBoundary1XGwEG3Z3z94qHL4'}
    payload = """------WebKitFormBoundary1XGwEG3Z3z94qHL4\r
Content-Disposition: form-data; name="uri"\r
\r
\r
------WebKitFormBoundary1XGwEG3Z3z94qHL4\r
Content-Disposition: form-data; name="username"\r
\r
ubnt\r
------WebKitFormBoundary1XGwEG3Z3z94qHL4\r
Content-Disposition: form-data; name="password"\r
\r
ubnt\r
------WebKitFormBoundary1XGwEG3Z3z94qHL4--\r
"""
    login_page_result = session.get('https://'+ip_address+'/index.cgi', verify=False)
    login_result = session.post('https://'+ip_address+'/login.cgi', headers=headers, data=payload, verify=False)
    status_result = session.get('https://'+ip_address+'/status.cgi', verify=False)
    ifstats_result = session.get('https://'+ip_address+'/iflist.cgi', verify=False)

    #print status_result.content
    status = status_result.json()
    wireless_status = status['wireless']
    host_status = status['host']

    ifstats = ifstats_result.json()

    status_msgs = []

    device_msg = diagnostic_updater.DiagnosticStatusWrapper()
    device_msg.name = 'Device'
    device_msg.summary(DiagnosticStatus.OK, host_status['hostname'])
    device_msg.add('Uptime', str(host_status['uptime']))
    device_msg.add('Role', host_status['netrole'])
    device_msg.add('Firmware Version', host_status['fwversion'])
    device_msg.add('Management IP Address', ip_address)
    status_msgs.append(device_msg)

    wireless_msg = diagnostic_updater.DiagnosticStatusWrapper()
    wireless_msg.name = 'Wireless'
    wireless_msg.summary(DiagnosticStatus.OK, human_wireless_mode(wireless_status['mode']) + ': ' + wireless_status['essid'])
    wireless_msg.add('AP MAC', wireless_status['apmac'])
    wireless_msg.add('Channel / Frequency', str(wireless_status['channel']) + ' / ' + wireless_status['frequency'])
    wireless_msg.add('Security', wireless_status['security'])
    status_msgs.append(wireless_msg)

    for iface_stats in ifstats['interfaces']:
        iface_msg = diagnostic_updater.DiagnosticStatusWrapper()
        iface_msg.name = iface_stats['ifname']
        if len(iface_stats['status']) > 0 and iface_stats['status'][0]['plugged'] == 1:
            iface_msg.summary(DiagnosticStatus.OK, human_bytes(iface_stats['stats']['rx_bytes']) + ' down / ' + human_bytes(iface_stats['stats']['tx_bytes']) + ' up')
        else:
            iface_msg.summary(DiagnosticStatus.WARN, 'Disconnected')
        iface_msg.add('Hardware Address', iface_stats['hwaddr'])
        for stat_name in iface_stats['stats']:
            iface_msg.add(stat_name.replace('_', ' ').title(), iface_stats['stats'][stat_name])
        status_msgs.append(iface_msg)


    for msg in status_msgs:
        msg.hardware_id = "Bullet: " + ip_address
        msg.name = device_name+': '+msg.name
    diagnostic_array = DiagnosticArray()
    diagnostic_array.status = status_msgs
    diagnostic_array.header.stamp = rospy.Time.now()
    diagnostic_pub.publish(diagnostic_array)


if __name__=='__main__':
    rospy.init_node("bullet_monitor")

    ip_address = rospy.get_param('~ip_address', '192.168.1.20')
    device_name = rospy.get_param('~device_name', rospy.get_name()[1:])

    period = rospy.get_param("~diagnostic_period", 5.0)

    timer = rospy.Timer(rospy.Duration(period), update_callback)
    diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size = 1)
    rospy.spin()
