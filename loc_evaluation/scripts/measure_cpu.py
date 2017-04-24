#!/usr/bin/env python
import sys
import rospy

import psutil
import os

from optparse import OptionParser

def getProcessIDs(name):
    proc_id = 0
    for proc in psutil.process_iter():
        try:
            if name in proc.name:
                proc_id = proc.pid
        except psutil.NoSuchProcess:
            pass

    return proc_id

if __name__ == '__main__':
    rospy.init_node('measure_cpu', anonymous=True)

    parser = OptionParser(usage="%prog -p or --process + process name", prog=os.path.basename(sys.argv[0]))

    parser.add_option("-p", "--process", dest="process", default=0, help="Process name")

    (options, args) = parser.parse_args()
    if not isinstance(options.process, basestring):
        parser.error("Please specify a process name using -p or --process")
        sys.exit()

    process_ID = getProcessIDs(options.process)
    if process_ID == 0:
        parser.error("No process named " + options.process)
    else:
        print "looking after CPU percentage of Process " + str(options.process) + " with ID " + str(process_ID)

    process = psutil.Process(process_ID)

    count = 0
    cpu = 0
    mem = 0
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()
        cpu = cpu + process.get_cpu_percent()
        mem = mem + process.get_memory_percent()
        count = count + 1
        if (count%10 == 0):
            print "average CPU percentage is: " + str(cpu/count)
            print "average Memory percentage is: " + str(mem/count)


