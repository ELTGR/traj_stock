#!/usr/bin/env python
import sys
import pathlib

import rospy
import smach
from time import sleep
import numpy as np
from Dynamic_Approach import traject3d
from Gridy_drone_swipp import Gridy_based

import math
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d   
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d
from matplotlib import animation

import sys
import pathlib

class Evit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'],
                            input_keys=['Point_Evit_IN1','Point_Evit_IN2'],
                            output_keys=['Point_Evit_OUT1','Point_Evit_OUT2'])
        self.counter = 0
	
    def execute(self, userdata):

        rospy.loginfo('Executing state Evit')
        rospy.loginfo('Point = ')   
        rospy.loginfo(userdata.Point_Evit_IN1)  

        pointxo= userdata.Point_Evit_IN1[0][0]
        pointyo= userdata.Point_Evit_IN1[1][0]
        
        #print('pointxo',pointxo)
        #print('pointyo',pointyo)


        pointx= userdata.Point_Evit_IN1[0][1]
        pointy= userdata.Point_Evit_IN1[1][1]
        pointz= userdata.Point_Evit_IN1[2][1]
        #print('pointx',pointx)
        #print('pointy',pointy)

        #print('pointz',pointz)       
        
        x,y,z = traject3d.main(pointx, pointy, pointz , pointxo, pointyo)



        for i in range(0,len(x)) :
            
            userdata.Point_Evit_IN2[0].append(x[i])
            userdata.Point_Evit_IN2[1].append(y[i])
            userdata.Point_Evit_IN2[2].append(z[i])




        PointEvit=[]
        Pointinter=[]
        if self.counter < 1:

            self.counter += 1
            return 'outcome1'


        else :
           # ob=traject3d.get_obtacle()
           # print('ob',ob)
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            print("Point_Evit_IN2",userdata.Point_Evit_IN2)
            ax.plot3D(userdata.Point_Evit_IN2[0], userdata.Point_Evit_IN2[1], userdata.Point_Evit_IN2[2], "-r")
            ax.axis("equal")
            ax.grid(True)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
            plt.show()
            sleep(1)
            plt.close()
            userdata.Point_Evit_OUT1=PointEvit 

            return 'outcome2'
 	    
 	    
 	    
 	    
class Scan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],
                            input_keys=['Point_Scan_IN1','Point_Scan_IN2'],
                            output_keys=['Point_Scan_OUT1','Point_Scan_OUT2'])
        

    def execute(self, userdata):
        rospy.loginfo('Executing state Scan')
        #rospy.loginfo('Point = ',userdata.Point_IN) 



        rospy.loginfo('Point = ')   
        rospy.loginfo(userdata.Point_Scan_IN1)   

        

        coefx=userdata.Point_Scan_IN1[0][1]
        coefy=userdata.Point_Scan_IN1[1][1]
        coefz=userdata.Point_Scan_IN1[2][1]
        ox = [0.0, 30.0, 30.0, 0.0, 0.0]
        x=np.ones(len(ox))*coefx
        ox=x+ox-2.5
        oy = [0.0, 0.0, 10.0, 10.0, 0.0]
        y=np.ones(len(oy))*coefy
        oy=y+oy-2.5
        oz = coefz
        oz=10
        resolution = 2
        #debut taille matrice  suivant taille marice +1 

        #for taille = taille px pas le truc de merde 
        # print('Point ox = ',ox)
        # print('Point oy = ',oy)   
        # print('Point oz = ',oz) 
        pz, py, px= Gridy_based.main(ox, oy, oz, resolution)

        for i in range(0, len(px)) :
            userdata.Point_Scan_IN2[0].append(px[i])
            userdata.Point_Scan_IN2[1].append(py[i])
            userdata.Point_Scan_IN2[2].append(pz[i])
    
        

        PointScan=[]
        for j in range(0,2):
            for i in range(0,3):
                PointScan = userdata.Point_Scan_IN1[i]
                PointScan = np.delete(PointScan, 0 )
                userdata.Point_Scan_IN1[i]=PointScan
                userdata.Point_Scan_OUT1=userdata.Point_Scan_IN1



        
        #rospy.loginfo('Point = ',userdata.Point_OUT) 
        userdata.Point_Scan_OUT1=userdata.Point_Scan_IN1

        return 'outcome1'

            
def main():
    

   
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    Rov = smach.StateMachine(outcomes=['outcome4'])
    Rov.userdata.WayPoints = [[0.0, 10, 35, 15], [0.0, 10, 15, 30], [10, 10, 10, 10]]
    Rov.userdata.traj = [[], [], []]
    #print("Rov.userdata.WayPoints",Rov.userdata.WayPoints[0][0])
    # Open the container
    with Rov:

        smach.StateMachine.add('Evit', Evit(), 
                            transitions={'outcome1':'Scan', 
                                         'outcome2':'outcome4'},
                            remapping = {'Point_Evit_IN1':'WayPoints', 
                                         'Point_Evit_IN2':'traj', 
                                         'Point_Evit_OUT1':'WayPoints',
                                         'Point_Evit_OUT2':'traj'})
        smach.StateMachine.add('Scan', Scan(), 
                            transitions={'outcome1':'Evit'},
                            remapping = {'Point_Scan_IN1':'WayPoints', 
                                         'Point_Scan_IN2':'traj', 
                                         'Point_Scan_OUT1':'WayPoints',
                                         'Point_Scan_OUT2':'traj'})


    # Execute SMACH plan
    outcome = Rov.execute()



if __name__ == '__main__':
    main()
