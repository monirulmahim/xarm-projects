import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from xarm_msgs.srv import MoveCartesian
from control_msgs.action import GripperCommand


XARM_SPEED = float(500)
XARM_ACC = float(5000)


class XArmController(Node):
    def __init__(self):
        super().__init__('xarm_controller')
        
        self.cartesian_client = self.create_client(MoveCartesian, '/xarm/set_position') 
        self.gripper_client = ActionClient(self, GripperCommand, '/xarm_gripper/gripper_action')
        
        self.get_logger().info('Waiting for services and action servers...')
        self.cartesian_client.wait_for_service()
        
        self.get_logger().info('Waiting for services and action servers for gripper...')
        self.gripper_client.wait_for_server()
        
        # This Small safety pauses (not for motion timing, just for mechanics)
        self.PRE_GRIP_PAUSE = 0.02
        self.POST_GRIP_PAUSE = 0.02
        

        # Motion blocking timeout (seconds). Increase if you move far.
        self.MOVE_TIMEOUT = 10.0
        
        self.get_logger().info("Ready")
        
    def move_cartesian(self, x: float, y: float, z: float) -> bool:
        req = MoveCartesian.Request() # Create a request object
        
        req.pose = [float(x), float(y), float(z), 3.14, 0.0, 0.0] # Set the desired pose (x, y, z, roll, pitch, yaw)
        req.speed = XARM_SPEED
        req.acc = XARM_ACC
        req.mvtime = 0.0 # Let the controller calculate the motion time based on speed and distance
        
        if hasattr(req, "wait"):
            req.wait = True
        if hasattr(req, "timeout"):
            req.timeout = float(self.MOVE_TIMEOUT)

        # Optional fields depending on branch/version
        if hasattr(req, "radius"):
            req.radius = float(15)
        if hasattr(req, "is_tool_coord"):
            req.is_tool_coord = False
        if hasattr(req, "relative"):
            req.relative = False
        if hasattr(req, "motion_type"):
            req.motion_type = 0  # default/linear planning

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res is None:
            self.get_logger().error("MoveCartesian call failed (no result).")
            return False

        # xArm returns ret/message
        if hasattr(res, "ret") and res.ret != 0:
            msg = getattr(res, "message", "")
            self.get_logger().error(f"MoveCartesian error ret={res.ret} msg='{msg}'")
            return False

        return True
    
    def gripper(self, position: float, max_effort: float = 0.0) -> bool:
        goal = GripperCommand.Goal()
        
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        time.sleep(self.POST_GRIP_PAUSE)
        return True
    
    def gripper_close(self) -> bool:
        return self.gripper(position=0.34, max_effort=0.0)

    def gripper_open(self) -> bool:
        return self.gripper(position=0.15, max_effort=0.0)
    
    def initial_position(self):
        self.move_cartesian(300, 0, 300) # initial position
        self.gripper_open()
    
    
    def run(self):

        zs_heights = [72, 166, 255, 345, 435, 525]

        self.initial_position()
        
        counter = 0
        cups = [
            
            # pickup_x, pickup_y, pickup_height, drop_x, drop_y, drop_height
            (190, 0, zs_heights[2], 400, -240, zs_heights[0]),
            (190, -200, zs_heights[1] , 400, -160, zs_heights[0]),
            (190, -120, zs_heights[1] , 400, -80, zs_heights[0]),
            (190, -40, zs_heights[1], 400, 0, zs_heights[0]),
            (190, 40, zs_heights[1], 400, 80, zs_heights[0]),
            (190, 120, zs_heights[1], 400, 160, zs_heights[0]),


            (190, 200, zs_heights[1], 400, -200, zs_heights[1]),
            (150, -240, zs_heights[0], 400, -120, zs_heights[1]),
            (150, -160, zs_heights[0], 400, -40, zs_heights[1]),
            (150, -80, zs_heights[0], 400, 40, zs_heights[1]),
            (150, 0, zs_heights[0], 400, 120, zs_heights[1]),


            (150, 80, zs_heights[0], 400, -160, zs_heights[2]),
            (150, 160, zs_heights[0], 400, -80, zs_heights[2]),
            (150, 240, zs_heights[0], 400, 0, zs_heights[2]),
            (224, -240, zs_heights[0], 400, 80, zs_heights[2]),



            (224, -160, zs_heights[0], 400, -120, zs_heights[3]),
            (224, -80, zs_heights[0], 400, -40, zs_heights[3]),
            (224, 0, zs_heights[0], 400, 40, zs_heights[3]),


            (224, 80, zs_heights[0], 400, -80, zs_heights[4]),
            (224, 160, zs_heights[0], 400, 0, zs_heights[4]),


            (224, 240, zs_heights[0], 400, -40, zs_heights[5]),
            
            ]
        
        
        for px, py, ph, dx, dy, dh in cups:
            
            # height_CH_Value = 350 if counter == 8 else 440 if counter == 13 else 300
            counter += 1
            height_CH_Value = {11: 370, 12: 370, 13: 440, 14: 470, 15: 480}.get(counter, 300)
            self.move_cartesian(px, py, height_CH_Value)
            self.move_cartesian(px, py, ph)
            self.gripper_close()
            self.move_cartesian(px, py, height_CH_Value)
            self.move_cartesian(dx, dy, height_CH_Value)
            self.move_cartesian(dx, dy, dh)
            self.gripper_open()
            self.move_cartesian(dx, dy, height_CH_Value)
            print(counter)
            
            #initial 210

        self.initial_position()

def main():
    rclpy.init()
    node  = XArmController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
