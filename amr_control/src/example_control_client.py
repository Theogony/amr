#! /usr/bin/env python
# remember to make this file executable (`chmod +x`) before trying to run it

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

# init node
rospy.init_node('motor_service_client')

# wait for this service to be running
rospy.wait_for_service('/amr_motor')

# Create the connection to the service. Remeber it's a Trigger service
motor_service = rospy.ServiceProxy('/amr_motor', SetBool)

bool_var = True

while(True):
    # Create an object of type SetBoolRequest
    req = SetBoolRequest(bool_var)

    # Now send the request through the connection
    result = motor_service(req)

    # Done, let's see the result!
    print (result)

    rospy.sleep(3)

    bool_var = not bool_var