#!/usr/bin/env python

import rospy
import psutil
import diagnostic_updater
from diagnostic_updater import DiagnosticStatus
import socket

def human_bytes(num_bytes):
    num_bytes = float(num_bytes)
    for x in ['B','KB','MB','GB']:
        if num_bytes < 1024:
            return "%3.1f %s" % (num_bytes, x)
        num_bytes /= 1024
    return "%3.1f%s" % (num_bytes, 'TB')


def cpu_task(stat):
    cpu_percent = psutil.cpu_percent(interval=0, percpu=True)
    total_cpu = str(psutil.cpu_percent(interval=0))+'%';
    stat.add('Total CPU', total_cpu)
    for i in range(len(cpu_percent)):
        stat.add('CPU '+str(i), str(cpu_percent[i])+'%')

    stat.summary(DiagnosticStatus.OK, total_cpu)

def virtual_memory_task(stat):
    virtual = psutil.virtual_memory()

    stat.summary(DiagnosticStatus.OK, str(virtual.percent)+'% Used')

    stat.add('Percent Used', str(virtual.percent)+'%')
    stat.add('Total Virtual Memory', human_bytes(virtual.total))
    stat.add('Available Virtual Memory', human_bytes(virtual.available))
    stat.add('Used Virtual Memory', human_bytes(virtual.used))
    stat.add('Free Virtual Memory', human_bytes(virtual.free))

def swap_memory_task(stat):
    swap = psutil.swap_memory()

    stat.summary(DiagnosticStatus.OK, str(swap.percent)+'% Used')

    stat.add('Total Swap Memory', human_bytes(swap.total))
    stat.add('Used Swap Memory', human_bytes(swap.used))
    stat.add('Free Swap Memory', human_bytes(swap.free))

def network_task(stat):
    net = psutil.net_io_counters()
    stat.add('Bytes Sent', human_bytes(net.bytes_sent))
    stat.add('Bytes Received', human_bytes(net.bytes_recv))
    stat.add('Packets Sent', net.packets_sent)
    stat.add('Packets Received', net.packets_recv)

    ifaces = psutil.net_io_counters(pernic=True)
    for iface in ifaces:
        iface_data = ifaces[iface]
        stat.add(iface+': Bytes Sent', human_bytes(iface_data.bytes_sent))
        stat.add(iface+': Bytes Received', human_bytes(iface_data.bytes_recv))
        stat.add(iface+': Packets Sent', iface_data.packets_sent)
        stat.add(iface+': Packets Received', iface_data.packets_recv)


    stat.summary(DiagnosticStatus.OK, human_bytes(net.bytes_sent)+' sent/'+human_bytes(net.bytes_recv)+' recv')


if __name__ == '__main__':
    hostname = socket.gethostname()

    if hostname:
        rospy.init_node('host_resource_monitor_'+hostname.replace('-', '_'))
    else:
        rospy.init_node('host_resource_monitor')

    updater = diagnostic_updater.Updater()

    updater.setHardwareID(hostname)

    updater.add('CPU', cpu_task)
    updater.add('Virtual Memory', virtual_memory_task)
    updater.add('Swap Memory', swap_memory_task)
    updater.add('Network', network_task)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        updater.update()
        r.sleep()
