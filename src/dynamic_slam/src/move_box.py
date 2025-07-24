#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose

class BoxMover(Node):
    def __init__(self):
        super().__init__('box_mover')
      
        self.start_y = -0.5        
        self.half_range = 0.5       
        self.step = 0.05           
        self.y = self.start_y      
        self.direction = -1        

     
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')

      
        self.timer = self.create_timer(0.1, self.move_callback)

    def move_callback(self):
       
        req = SetEntityState.Request()
        state = EntityState()
        state.name = 'moving_box'
        state.reference_frame = 'world'

        pose = Pose()
        pose.position.x = -0.5
        pose.position.y = self.y
        pose.position.z = 0.0
        state.pose = pose
        req.state = state

        self.cli.call_async(req)

    
        self.y += self.step * self.direction

       
        if self.y > self.start_y + self.half_range:
            self.y = self.start_y + self.half_range
            self.direction = -1
        elif self.y < self.start_y - self.half_range:
            self.y = self.start_y - self.half_range
            self.direction = 1

    def _result_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error('SetEntityState failed: ' + res.status_message)
        except Exception as e:
            self.get_logger().error(f'Service exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BoxMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
