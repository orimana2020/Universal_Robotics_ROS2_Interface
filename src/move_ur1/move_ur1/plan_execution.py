import rclpy
from rclpy.node import Node
import numpy as np
import time
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import SetIO
from control_msgs.srv import QueryTrajectoryState

class UR_Node(Node):
    def __init__(self):
        super().__init__('move_ur5_joint_trajectory')
        self.trajectory_publisher = self.create_publisher(JointTrajectory, 
                                                        '/joint_trajectory_controller/joint_trajectory'
                                                        , 10)
        self.set_tool_clie = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.set_tool_clie.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.tool_req = SetIO.Request()
        
        self.state_clie = self.create_client(QueryTrajectoryState, '/joint_trajectory_controller/query_state')
        while not self.state_clie.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_state_req = QueryTrajectoryState.Request()        

        self.max_angular_velocity = np.pi # [rad/sec]
        
        plan_list = ['open', 'cube_1_0_path.npy', 'close', 'cube_1_1_path.npy',
                      'open', 'cube_2_0_path.npy', 'close', 'cube_2_1_path.npy',
                      'open', 'cube_3_0_path.npy', 'close', 'cube_3_1_path.npy',
                      'open', 'cube_4_0_path.npy', 'close', 'cube_4_1_path.npy',
                      'open', 'cube_5_0_path.npy', 'close', 'cube_5_1_path.npy',
                      'open', 'cube_6_0_path.npy', 'close', 'cube_6_1_path.npy',
                      'open', ]  

        self.plan = []
        for path_name in plan_list:
            if( path_name == 'open' or path_name =='close'):
                self.plan.append(path_name)
                continue
            self.plan.append(np.load('./src/move_ur1/motion_planning_lab_python_interface_SOLVED/task3_path_pyramid/'+path_name)) 
            # self.plan.append(np.load(path_name)) 

        self.execute_plan()

    def execute_plan(self):
        for path_idx in range(len(self.plan)):
            if type(self.plan[path_idx]) == str:
                if self.plan[path_idx] == 'open':
                    self.send_request(0)
                    print('Tool open')
                if self.plan[path_idx] == 'close':
                    self.send_request(1)
                    print('Tool close')
                time.sleep(1)
            else:
                self.send_path(self.plan[path_idx])
    
    def get_current_state(self):
        self.get_state_req.time = self.get_clock().now().to_msg()
        print(self.get_state_req.time)
        self.future_state = self.state_clie.call_async(self.get_state_req)
        rclpy.spin_until_future_complete(self, self.future_state)
        return self.future_state.result().position


    def wait_until_reach_goal(self, goal):
        time.sleep(2)
        current_state = self.get_current_state()
        if len(current_state) != 6:
            print('try service call again')
            diff = 10
        else:
            diff  = np.sum(np.abs(goal - current_state))
        print('waiting to reach goal...')
        while diff > 0.1:
            time.sleep(2)
            current_state = self.get_current_state()
            if len(current_state) != 6:
                print('try service call again')
                continue
            diff  = np.sum(np.abs(goal - current_state))
        time.sleep(0.5)
            
            
    
    def send_path(self, path):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        msg.header.stamp =self.get_clock().now().to_msg()
        time_stamp_sec = 0
        time_stamp_nanosec = 0
        for state_idx in range(1, len(path)):
            point = JointTrajectoryPoint()
            point.positions = list(path[state_idx])
            time_from_start = Duration()
            max_angle_diff = np.max(np.abs(path[state_idx] - path[state_idx - 1]))
            delta_t_sec = int(max_angle_diff // self.max_angular_velocity) 
            delta_t_nanosec = (max_angle_diff / self.max_angular_velocity - delta_t_sec) * 1e9
            if delta_t_sec == 0:
                delta_t_nanosec = max(1e9* 0.8, delta_t_nanosec)
            
            time_stamp_nanosec = time_stamp_nanosec + delta_t_nanosec
            if time_stamp_nanosec >= 1e9:
                time_stamp_nanosec -= 1e9
                delta_t_sec += 1
            
            time_from_start.sec = delta_t_sec + time_stamp_sec
            
            time_from_start.nanosec = int(time_stamp_nanosec)

            time_stamp_sec = time_from_start.sec
            time_stamp_nanosec = time_from_start.nanosec
              
            point.time_from_start = time_from_start     
            msg.points.append(point)
        self.trajectory_publisher.publish(msg)
        print(msg)
        print('Trajectory published')
        self.wait_until_reach_goal(path[-1])


    def send_request(self, state):
        self.tool_req.fun = 1
        self.tool_req.pin = 0
        self.tool_req.state = float(state)
        self.future = self.set_tool_clie.call_async(self.tool_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    

def main(args=None):
    rclpy.init(args=args)
    pub_node = UR_Node()    
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()