from pydantic import BaseModel
from typing import Literal


class TopicSpec(BaseModel):
    name: str
    msg_type: str


class CameraTopics(BaseModel):
    k4a_rgb: TopicSpec = TopicSpec(
        name="/k4a/rgb/image_raw", msg_type="sensor_msgs/msg/Image"
    )
    realsense_color: TopicSpec = TopicSpec(
        name="/camera/camera/color/image_raw", msg_type="sensor_msgs/msg/Image"
    )
    secondary_color: TopicSpec = TopicSpec(
        name="/camera/color/image_raw", msg_type="sensor_msgs/msg/Image"
    )

class ArmTopics(BaseModel):
    joint_trajectory: TopicSpec = TopicSpec(
        name="/joint_trajectory_controller/joint_trajectory", msg_type="trajectory_msgs/JointTrajectory"
    )
    publish_test: TopicSpec = TopicSpec(
        name="/publish_test", msg_type="trajectory_msgs/JointTrajectory"
    )

class FeedbackTopics(BaseModel):
    ha_lights: TopicSpec = TopicSpec(
        name="/feedback/ha_lights", msg_type="std_msgs/msg/String"
    )
    audio_tts: TopicSpec = TopicSpec(
        name="/tts/generate", msg_type="std_msgs/msg/String"
    )
    audio_set_voice: TopicSpec = TopicSpec(
        name="/tts/set_voice", msg_type="std_msgs/msg/String"
    )

class TopicList(BaseModel):
    camera: CameraTopics = CameraTopics()
    arm: ArmTopics = ArmTopics()
    feedback: FeedbackTopics = FeedbackTopics()