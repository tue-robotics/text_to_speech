#!/usr/bin/env python
# coding: utf-8

'''
This node listens to a service call and a topic for text to speech
requests. These will be processed by the festival or the philips tts module.
'''

import rospy
import os

from std_srvs.srv import Empty
from text_to_speech.srv import Play

class Player(object):

    def __init__(self):
        # services
        self.srv_play         = rospy.Service('~play',         Play,      self.play_srv)
        self.srv_clear_buffer = rospy.Service('~clear_buffer', Empty,     self.clear_buffer_srv)

        # buffer audio requests
        self.buffer = []
        self.is_playing = False

    def play_srv(self, req):
        if req.audio_type not in ["wav", "mp3", "oga"]:
            msg = "Audio format '%s' not supported" % req.audio_type
            rospy.logerr(msg)
            return msg

        self.buffer += [req]

        if req.blocking_call:
            while not rospy.is_shutdown() and self.buffer:
                self.step()
                rospy.sleep(0.01)

        return ""

    def play(self, req):
        self.is_playing = True

        import time
        filename = time.strftime("/tmp/%Y-%m-%d-%H-%M-%S") + "." + req.audio_type

        with open(filename, "wb") as f:
            f.write(bytearray(req.audio_data))

        err_code = os.system("play %s pitch %i > /dev/null 2>&1" % (filename, req.pitch))

        if err_code:
            rospy.logerr("Could not play %s: return code: %i" % (filename, err_code))

        self.is_playing = False

    def step(self):
        if not self.buffer or self.is_playing:
            return

        # Play first request from the buffer
        self.play(self.buffer[0])

        # If the buffer was not emptied in the meantime, pop the first request
        if self.buffer:
            self.buffer.pop(0)

    def clear_buffer_srv(self, req):
        self.buffer = []
        return []

if __name__ == "__main__":
    rospy.init_node('audio_player')

    player = Player()

    while not rospy.is_shutdown():
        player.step()
        rospy.sleep(0.1)
