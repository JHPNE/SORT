import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional, Type
import time
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerPublisher import TopicHandlerPublisher
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber
 
from .FeedBackDecisionHandler import (SortState, Verdict,
                                      FeedBackDecisionHandler, parse_state)
from vision_module.Zone import ZONES
from vision_module.WorldClient import TagWorld
 
from control_module.GestureClient import GestureClient

# How often the tag world is judged, in seconds.
DECISION_PERIOD_S = 1.0

# Home Assistant light entity used for the visual feedback.
LIGHT_ENTITY_ID = 'light.robolab'


class FeedBackGesture(ABC):
 
    def __init__(self, client: GestureClient):
        self.client = client
 
    @abstractmethod
    def handle(self) -> None:
        ...


class PositiveFeedBackGesture(FeedBackGesture):
    def handle(self) -> None:
        self.client.nod()
 
 
class NegativeFeedBackGesture(FeedBackGesture):
    def handle(self) -> None:
        self.client.shake()
 
 
class UnknownFeedBackGesture(FeedBackGesture):
    def handle(self) -> None:
        return


@dataclass(frozen=True)
class FeedbackReaction:
    tts_message: str
    light_color: str
    gesture: Type[FeedBackGesture]


RESULT_FEEDBACK: Dict[SortState, FeedbackReaction] = {
    SortState.CORRECT: FeedbackReaction(
        'You sorted it correctly. Well done!', 'green',
        PositiveFeedBackGesture),
    SortState.INCORRECT: FeedbackReaction(
        'You sorted it wrong. You can do better!', 'red',
        NegativeFeedBackGesture),
    SortState.UNKNOWN: FeedbackReaction(
        "I do not know where that one belongs. You decide!", 'yellow',
        UnknownFeedBackGesture),
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

        self._gestures = GestureClient(self)

        # Remember the last verdict so the same feedback is not repeated every tick.
        self._last_key: Optional[tuple] = None

        self._world = TagWorld(self)
        self._decision_handler = FeedBackDecisionHandler(self._world, zones=ZONES)

        self.create_timer(DECISION_PERIOD_S, self._on_decision_tick)
        self.get_logger().info(f'Judging the tag world every {DECISION_PERIOD_S}s')

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

    def _on_decision_tick(self):
        """Ask the decision handler what the current tag world means."""
        if self._gestures.get_is_currently_moving():
            return

        verdict: Verdict = self._decision_handler.evaluate()

        if not verdict.sensing_ok:
            # Blind, not uncertain. Say nothing, keep the last light.
            self.get_logger().debug(f'no judgement: {verdict.reason}')
            self._last_key = None      # re-announce once we can see again
            return
 
        if verdict.implausible:
            self.get_logger().warning(
                f'implausible cube poses {verdict.implausible} - check the '
                f'tag sizes in TagRegistry')
 
        key = verdict.key()
        if key == self._last_key:
            return
        self._last_key = key
 
        self.get_logger().info(
            f'Sorting state: {verdict.state.name} ({verdict.reason})')
        self._react_to_result(verdict.state)

    def _on_sorting_result(self, msg: String):
        """React to a sorting outcome published by hand or by another node."""
        if self._gestures.get_is_currently_moving():
            return

        state = parse_state(msg.data)
        if state is None:
            self.get_logger().warning(f'Unknown sorting result: "{msg.data}"')
            return
        self._react_to_result(state)

    def _react_to_result(self, result: SortState):
        """Give the speech and light feedback belonging to one sorting result."""
        reaction = RESULT_FEEDBACK.get(result)
        if reaction is None:
            self.get_logger().warning(f'Unknown sorting result: "{result}"')
            return

        self.give_feedback(
            light={'entity_id': LIGHT_ENTITY_ID, 'action': 'turn_on',
                   'color_name': reaction.light_color},
            text=reaction.tts_message,
            gesture=reaction.gesture(self._gestures),
        )

    def give_feedback(self, light: dict = None, text: str = None, gesture: str = None):
        """Convenience helper to trigger multiple feedback modalities at once."""
        if light:
            self.set_light(**light)
        if gesture:
            self.play_gesture(gesture, text)

    def play_gesture(self, gesture: FeedBackGesture, text: str = None):
        """Publish a named arm gesture preset as a JointTrajectory."""
        try:
            gesture.handle()
            time.sleep(5)
            self.speak(text)
        except Exception:
            self.get_logger().exception(
                f'gesture {type(gesture).__name__} failed')


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
