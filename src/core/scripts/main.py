import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine
from std_msgs.msg import String
from time import sleep
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

frame_ = Image()
bridge = CvBridge()

target = ''
index = 0
last_patrol = 'p1'

waypoints = [
    ['p1', (8.4, -7.0), (0.0, 0.0, 0.0, 1.0)],
    ['p2', (7.44, -2.85), (0.0, 0.0, 0.0, 1.0)],
    ['p3', (-6.79, -4.14), (0.0, 0.0, 0.0, 1.0)]
]

offices = [
    ['o1', (-6.31, -3.84), (0.0, 0.0, 0.0, 1.0)],
    ['o2', (-5.8, -0.5), (0.0, 0.0, 0.0, 1.0)],
    ['o3', (2.46, -0.2), (0.0, 0.0, 0.0, 1.0)],
    ['o4', (7.84, -1.62), (0.0, 0.0, 0.0, 1.0)],
    ['o5', (2.35, -4.61), (0.0, 0.0, 0.0, 1.0)],
    ['o6', (8.48, -6.77), (0.0, 0.0, 0.0, 1.0)]
]


patrol = StateMachine('success')

class Delivery(State):
    def __init__(self):
        State.__init__(self, outcomes=['p1', 'p2', 'p3'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'

    
    def execute(self, userdata):
        global last_patrol
        global index
        self.goal.target_pose.pose.position.x = offices[index][1][0]
        self.goal.target_pose.pose.position.y = offices[index][1][1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = offices[index][2][0]
        self.goal.target_pose.pose.orientation.y = offices[index][2][1]
        self.goal.target_pose.pose.orientation.z = offices[index][2][2]
        self.goal.target_pose.pose.orientation.w = offices[index][2][3]
        print('Delivering to office', offices[index][0])
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return last_patrol


class Waypoint(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'delivery'])
        # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]
        
    def execute(self, userdata):
        global target
        self.client.send_goal(self.goal)
        while self.client.get_state() == 1 or self.client.get_state() == 0:            
            if target != '':
                self.client.cancel_goal()
                print('Delivery requested to office', target)
                target = ''
                return 'delivery'

        return 'success'
    

def callback(data):
    global last_patrol
    global index
    global target
    # Find the target office
    print('Looking for office', target)
    for i,o in enumerate(offices):
        if str(o[0]).lower() == str(data.data).lower():
            index = i
            print( 'Target office:', target, 'Index:', index)
            last_patrol = patrol.get_active_states()[0]
            print('Last patrol:', last_patrol)
            break
    target = data.data 
    
def camera_cb(data):
    global frame_, bridge
    frame_ = data
    try:
        cv_image = bridge.imgmsg_to_cv2(frame_, "bgr8")
        cv_image = cv.resize(cv_image, (640, 480))
        cv.imshow("Robot Camera", cv_image)
        cv.waitKey(3)
    except CvBridgeError as e:
        print(e)

    
rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)
    

if __name__ == "__main__":
    rospy.init_node('patrol')
    rospy.Subscriber('delivery', String, callback)

    
    with patrol:
        for i,w in enumerate(waypoints):
            StateMachine.add(w[0], Waypoint(w[1], w[2]), transitions={'success':waypoints[(i + 1) % len(waypoints)][0], 'delivery': 'delivery'})
        StateMachine.add('delivery', Delivery(), transitions={'p1': 'p1', 'p2': 'p2', 'p3': 'p3'})
    print('Runing patrol...')
    patrol.execute()
    
