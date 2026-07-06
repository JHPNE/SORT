from pydantic import BaseModel
from typing import Literal


class TopicSpec(BaseModel):
    name: str
    msg_type: str


class CameraTopics(BaseModel):
    rgb_image_raw: TopicSpec = TopicSpec(
        name="/k4a/rgb/image_raw", msg_type="sensor_msgs/msg/Image"
    )


class ArmTopics(BaseModel):
    pass


class TopicList(BaseModel):
    camera: CameraTopics = CameraTopics()