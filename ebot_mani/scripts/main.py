#!/usr/bin/env python

# Import
import rospy

# Import from files
from settings import Settings
from flags import Flags
from camera import Camera


def main():
    rospy.init_node(name='sara')
    settings = Settings()
    flags = Flags()
    camera = Camera(
        settings=settings.camera,
        flags=flags
    )
    rate = rospy.Rate(hz=1)
    rate.sleep()
    rospy.loginfo('Start')
    while True:
        time = rospy.Time.now()
        flags.update()
        camera.pc.preprocess()
        camera.pc.localize_clusters()
        rospy.loginfo('Time taken: ' + str((rospy.Time.now() - time)/1e6))
        rate.sleep()    


if __name__=='__main__':
    main()