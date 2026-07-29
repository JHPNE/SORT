import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerPublisher import TopicHandlerPublisher
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber


# The vision pipeline is optional: without it the node still reacts to messages
# published on /sorting/result by hand, which is how audio and light are tested.
try:
    from vision_module.WorldClient import TagWorld
    from FeedbackController.FeedBackDecisionHandler import (
        FeedBackDecisionHandler, SortState)
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False

# How often the tag world is judged, in seconds.
DECISION_PERIOD_S = 1.0

# Named arm gestures: joint_1..joint_6 target positions (radians), duration in seconds.
# CHANGE / EXTEND PRESETS HERE
ARM_GESTURES = {
    'nod': ([0.0, -0.3, 0.3, 0.0, 0.0, 0.0], 2),
    'wave': ([0.0, 0.0, -0.4, 0.4, 0.0, 0.0], 2),
    'home': ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3),
}

# Home Assistant light entity used for the visual feedback.
LIGHT_ENTITY_ID = 'light.robolab'

# Speech + light colour for each /sorting/result value published by the vision
# pipeline. The bridge only accepts the colours green, yellow and red.
SORTING_RESULT_FEEDBACK = {
    'correct': ('You sorted it correctly. Well done!', 'green'),
    'incorrect': ('You sorted it wrong. You can do better!', 'red'),
    'uncertain': ('I am uncertain with your sorting. Please double check it.', 'yellow'),
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

        self._sorting_result_sub = TopicHandlerSubscriber(
            node=self, topic_spec=topics.sorting.result,
            callback=self._on_sorting_result)

        # Remember the last verdict so the same feedback is not repeated every tick.
        self._last_state = None

        self._decision_handler = None
        if VISION_AVAILABLE:
            self._world = TagWorld(self)
            self._decision_handler = FeedBackDecisionHandler(self._world)
            self.create_timer(DECISION_PERIOD_S, self._on_decision_tick)
            self.get_logger().info(
                f'Judging the tag world every {DECISION_PERIOD_S}s')
        else:
            self.get_logger().warning(
                'vision_module not found, only reacting to /sorting/result')

    # light 
    def set_light(self, entity_id: str, action: str = 'turn_on', **params):
        """Publish a Home Assistant light command as JSON for a HA bridge node to consume."""
        payload = {'entity_id': entity_id, 'action': action, **params}
        msg = String()
        msg.data = json.dumps(payload)
        self.get_logger().info(f'HA light command: {msg.data}')
        self._ha_lights_pub.publish(msg)

    # text for tts
    def speak(self, text: str):
        """Publish raw text to be spoken via TTS on the audio speaker node."""
        msg = String()
        msg.data = text
        self.get_logger().info(f'Audio feedback: {msg.data}')
        self._audio_pub.publish(msg)

    # voice for tts
    def set_voice(self, voice: str):
        """Publish a voice name to /tts/set_voice before the next speak() call."""
        msg = String()
        msg.data = voice
        self.get_logger().info(f'TTS voice set to: {msg.data}')
        self._audio_voice_pub.publish(msg)

    # feedback gesture
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

    def _on_decision_tick(self):
        """Ask the decision handler what the current tag world means."""
        verdict = self._decision_handler.evaluate()

        # Only react when the situation actually changed, otherwise the same
        # sentence would be repeated every DECISION_PERIOD_S seconds.
        if verdict.state == self._last_state:
            return
        self._last_state = verdict.state

        self.get_logger().info(
            f'Sorting state: {verdict.state.name} ({verdict.reason})')

        # UNKNOWN means FeedbackSilent: not enough information to judge, so the
        # robot keeps quiet instead of guessing.
        if verdict.state == SortState.UNKNOWN:
            return

        result = 'correct' if verdict.state == SortState.CORRECT else 'incorrect'
        self._react_to_result(result)

    def _on_sorting_result(self, msg: String):
        """React to a sorting outcome published by hand or by another node."""
        self._react_to_result(msg.data)

    def _react_to_result(self, result: str):
        """Give the speech and light feedback belonging to one sorting result."""
        feedback = SORTING_RESULT_FEEDBACK.get(result)
        if feedback is None:
            self.get_logger().warning(f'Unknown sorting result: "{result}"')
            return

        text, color = feedback
        self.give_feedback(
            light={'entity_id': LIGHT_ENTITY_ID, 'action': 'turn_on', 'color_name': color},
            text=text,
            # gesture='nod' # noch nicht implemented und getestet
        )


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
