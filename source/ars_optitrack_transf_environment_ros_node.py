#!/usr/bin/env python3


from ars_optitrack_transf_environment_ros import *


def main():
    ars_optitrack_transf_environment_ros = ArsOptitrackTransfEnvironmentRos()

    ars_optitrack_transf_environment_ros.init()

    ars_optitrack_transf_environment_ros.open()

    try:
        ars_optitrack_transf_environment_ros.run()
    except rospy.ROSInterruptException:
        pass

    return 0


if __name__ == "__main__":
    main()
