#!/usr/bin/env python3
import rospy

from custom_msgs.msg import Obstacles, Zone, Form
from geometry_msgs.msg import Point



if __name__ == '__main__':
    rospy.init_node('broadcast_obstacle_node', anonymous=False)
    rospy.sleep(0.2)  # Wait for a while for init to complete before printing.
    rospy.loginfo(rospy.get_name() + " start")

    obstacle_sub = rospy.Publisher('/vision_costmap_layer/obsctacles', Obstacles, queue_size=10)
    # obstacle_sub = rospy.Publisher('/virtual_costamp_layer/obsctacles', Obstacles, queue_size=10)

    data = Obstacles()
    pt = Point()
    # pt.x = 1
    # pt.y = 2
    # tmp = Form()
    # tmp.form.append(pt)
    # pt.x = 1
    # pt.y = 2
    # tmp.form.append(pt)
    # data.list.append(tmp)
    
    
    pt.x = 3
    pt.y = 4
    pt.z = 1.0
    tmp2 = Form()
    tmp2.form.append(pt)
    data.list.append(tmp2)

    while not rospy.is_shutdown():
        print('Publishing ...')
        obstacle_sub.publish(data)
        rospy.sleep(.5)
        # rospy.spinOnce()
    # rospy.spin()


