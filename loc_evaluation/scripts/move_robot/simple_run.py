#!/usr/bin/python
import rospy
from simple_script_server import *

class simple_run():
    def __init__(self):
        self.sss = simple_script_server()
        self.station1 = [40, -3, 1.57]
        self.station2 = [40, -10, -1.57]
        self.station3 = [39.2, -23, 0]
        self.num_iterations = 10

    def gotoStation1(self):
        print "go to station1"        
        self.sss.move("base", self.station1)

    def gotoStation2(self):
        print "go to station2"              
        self.sss.move("base", self.station2)

    def gotoStation3(self):
        print "go to station3"              
        self.sss.move("base", self.station3)

    def main(self):

        for i in range(self.num_iterations):
            print i, '. run!'
            self.gotoStation1()
            #rospy.sleep(1.0)
            self.gotoStation2()
            #rospy.sleep(1.0)
            self.gotoStation3()         

if __name__ == '__main__':
    rospy.init_node("simple_run")
    sr = simple_run()
    sr.main()
