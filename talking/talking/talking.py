#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from playsound import playsound
from robp_interfaces.srv import Talk
from std_msgs.msg import Empty
from typing import Final

import os


class Talking(Node):
    SOUND_PATH: Final[str] = '/home/group7/sounds/'
    SOUND_SUFFIX: Final[str] = '.mp3'

    def __init__(self):
        super().__init__('talking')

        self.srv = self.create_service(Talk, 'talk', self.talk_callback)

    def talk_callback(self, request: Talk.Request, response: Talk.Response):

        try:
            sound_path = os.path.join(
                self.SOUND_PATH, request.sound.data + self.SOUND_SUFFIX)

            self.get_logger().info(f"Playing sound: {sound_path}")

            # make this async
            playsound(sound_path, block=False)
            response.empty = Empty()
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        self.get_logger().info("Sound played")
        return response


def main(args=None):
    rclpy.init(args=args)

    talking = Talking()

    try:
        rclpy.spin(talking)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
