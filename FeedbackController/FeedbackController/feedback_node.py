import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerPublisher import TopicHandlerPublisher

# Named arm gestures: joint_1..joint_6 target positions (radians), duration in seconds.
# CHANGE / EXTEND PRESETS HERE
ARM_GESTURES = {
    'nod': ([0.0, -0.3, 0.3, 0.0, 0.0, 0.0], 2),
    'wave': ([0.0, 0.0, -0.4, 0.4, 0.0, 0.0], 2),
    'home': ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3),
}


class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')

        topics = TopicList()

        self._ha_lights_pub = TopicHandlerPublisher(
            node=self, topic_spec=topics.feedback.ha_lights)
        self._audio_pub = TopicHandlerPublisher(
            node=self, topic_spec=topics.feedback.audio_tts)
        self._audio_voice_pub = TopicHandlerPublisher(
            node=self, topic_spec=topics.feedback.audio_set_voice)
        self._arm_pub = TopicHandlerPublisher(
            node=self, topic_spec=topics.arm.joint_trajectory)

    def set_light(self, entity_id: str, service: str = 'turn_on', **params):
        """Publish a Home Assistant light command as JSON for a HA bridge node to consume."""
        payload = {'entity_id': entity_id, 'service': service, 'params': params}
        msg = String()
        msg.data = json.dumps(payload)
        self.get_logger().info(f'HA light command: {msg.data}')
        self._ha_lights_pub.publish(msg)

    def speak(self, text: str):
        """Publish raw text to be spoken via TTS on the audio speaker node."""
        msg = String()
        msg.data = text
        self.get_logger().info(f'Audio feedback: {msg.data}')
        self._audio_pub.publish(msg)

    def set_voice(self, voice: str):
        """Publish a voice name to /tts/set_voice before the next speak() call."""
        msg = String()
        msg.data = voice
        self.get_logger().info(f'TTS voice set to: {msg.data}')
        self._audio_voice_pub.publish(msg)

    def play_gesture(self, name: str):
        """Publish a named arm gesture preset as a JointTrajectory."""
        if name not in ARM_GESTURES:
            self.get_logger().error(
                f'Unknown gesture "{name}". Available: {list(ARM_GESTURES)}')
            return

        joint_positions, duration = ARM_GESTURES[name]

        msg = JointTrajectory()
        msg.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = duration
        msg.points.append(point)

        self.get_logger().info(f'Arm gesture "{name}": {msg}')
        self._arm_pub.publish(msg)

    def give_feedback(self, *, light: dict = None, text: str = None, gesture: str = None):
        """Convenience helper to trigger multiple feedback modalities at once."""
        if light:
            self.set_light(**light)
        if text:
            self.speak(text)
        if gesture:
            self.play_gesture(gesture)


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackNode()

    # Example: greet the user with light, speech and an arm gesture.
    node.give_feedback(
        light={'entity_id': 'light.living_room', 'service': 'turn_on', 'color_name': 'blue'},
        text='Hello, this is your robot assistant speaking!',
        gesture='wave',
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
