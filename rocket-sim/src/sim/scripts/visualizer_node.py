import rclpy
from rclpy.node import Node
from vtvl_msgs.msg import VehicleState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.state_sub = self.create_subscription(
            VehicleState,
            '/vehicle/state',
            self.state_callback,
            10)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo\'s /set_entity_state service not available, waiting...')

    def state_callback(self, msg: VehicleState):
        pose = Pose()
        pose.position = msg.position
        pose.orientation = msg.attitude

        request = SetEntityState.Request()
        request.state.name = 'vtvl_rocket'
        request.state.pose = pose

        self.set_state_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()