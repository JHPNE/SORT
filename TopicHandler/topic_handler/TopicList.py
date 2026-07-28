from pydantic import BaseModel
from typing import Literal


class TopicSpec(BaseModel):
    name: str
    msg_type: str


class CameraTopics(BaseModel):
    arm_camera: TopicSpec = TopicSpec(
        name="/k4a/rgb/image_raw", msg_type="sensor_msgs/msg/Image"
    )
    realsense_color: TopicSpec = TopicSpec(
        name="/camera/camera/color/image_raw", msg_type="sensor_msgs/msg/Image"
    )
    secondary_color: TopicSpec = TopicSpec(
        name="/camera/color/image_raw", msg_type="sensor_msgs/msg/Image"
    )


class ArmTopics(BaseModel):
    pass


class TopicList(BaseModel):
    camera: CameraTopics = CameraTopics()