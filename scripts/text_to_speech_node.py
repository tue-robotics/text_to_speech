#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This node listens to a service call and a topic for text to speech
requests. These will be processed by the festival or the philips tts module.
"""

import rospy
import os

from subprocess import Popen, PIPE
from std_msgs.msg import String
from text_to_speech.srv import Speak, SpeakRequest, Play, PlayRequest
import re
import sys

reload(sys)
sys.setdefaultencoding('utf8')


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class TTS(object):
    def __init__(self, tts_module, executable=None, key=None, pitch=0):
        # philips specific arguments
        self.executable = executable
        self.key = key
        self.pitch = pitch

        self.character = rospy.get_param("~character", "Default")
        self.language = rospy.get_param("~language", "us")
        self.voice = rospy.get_param("~voice", "kyle")
        self.emotion = rospy.get_param("~emotion", "Neutral")
        self.samples_path = rospy.get_param("~samples_path", "~/MEGA/media/audio/soundboard")

        # topics
        self.sub_speak = rospy.Subscriber("~input", String, self.speak)
        self.pub_speak = rospy.Publisher("~output", String, queue_size=10)

        # services
        self.srv_speak = rospy.Service('~speak',        Speak,     self.speak_srv)

        # clients
        self.client_play = rospy.ServiceProxy('play', Play)

        # the current active module (philips or festival)
        self.tts_module = tts_module

    def do_tts(self, req):
        self.pub_speak.publish(req.sentence)

        rospy.loginfo('TTS: "' + bcolors.OKBLUE + req.sentence + bcolors.ENDC + '"')
        rospy.loginfo(
            "TTS: '{0}' (module: '{1}', character: '{2}', language: '{3}', voice: '{4}', emotion: '{5}')"
            .format(req.sentence, self.tts_module, req.character, req.language, req.voice, req.emotion)
        )

        # Check if an audio file for this sentence already exists
        for extension in ["wav", "mp3", "oga"]:
            potential_filename = os.path.join(os.path.expanduser(self.samples_path), req.sentence.lower() + "." + extension)
            rospy.logdebug("Checking for file on path: " + potential_filename)
            if os.path.isfile(potential_filename):
                rospy.logdebug("Found file!")
                play_req = PlayRequest()
                play_req.audio_data = open(potential_filename, "rb").read()
                play_req.audio_type = extension
                play_req.blocking_call = req.blocking_call
                play_req.pitch = 0
                resp = self.client_play(play_req)
                rospy.logdebug("Response: " + resp.error_msg)
                return ""

        # No audio sample existed, continuing with TTS
        if self.tts_module == "philips":
            return self.do_tts_philips(req)
        else:
            return self.do_tts_festival(req)

    def do_tts_philips(self, req):
        text = "¬<" + req.character + ">" + '\n'  # Add character
        text += "¬<speaker=" + req.language + "_" + req.voice + ">" + '\n'  # Add language + voice
        text += "¬<" + req.emotion + ">" + req.sentence

        tmp_text_file = "/tmp/temp_speech_text.txt"

        tts_file = file(tmp_text_file, "w")
        tts_file.write(text.encode("utf8"))
        tts_file.close()

        rospy.loginfo("File created: %s" % tmp_text_file)

        save_name = re.sub(r'(\W+| )', '', req.sentence)
        if not save_name:
            save_name = "empty"

        save_filename = '/tmp/%s.wav' % save_name
        command = self.executable + " -i {0} -k ".format(tmp_text_file) + self.key + " -o {0}".format(save_filename)
        rospy.loginfo("Executing cmd: %s", command)

        p = Popen(command, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p.communicate()
        rc = p.returncode

        if rc != 0:
            rospy.logerr("Return code TTS Philips != 0: %s", stderr)
            return False

        rospy.loginfo(".wav file created: {0}".format(save_filename))

        if not os.path.isfile(save_filename):
            rospy.logerr("Cannot create wav file for request: {}".format(req))
            return False

        # Play sound
        play_req = PlayRequest()
        play_req.audio_data = open(save_filename, "rb").read()
        play_req.audio_type = "wav"
        play_req.blocking_call = req.blocking_call
        play_req.pitch = 0
        resp = self.client_play(play_req)

        if resp.error_msg:
            rospy.logerr("Could not play sound: %s", resp.error_msg)

        rospy.loginfo(".wav file played, removing temporary files")

        return True

    def do_tts_festival(self, req):
        filename = "/tmp/festival.wav"

        command = "echo \"%s\" | text2wave -o %s" % (req.sentence, filename)
        rospy.loginfo(command)

        p = Popen(command, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p.communicate()
        rc = p.returncode

        if stdout:
            rospy.logwarn(stdout)
        rospy.loginfo(stderr)

        # Play sound
        play_req = PlayRequest()
        play_req.audio_data = open(filename, "rb").read()
        play_req.audio_type = "wav"
        play_req.blocking_call = req.blocking_call
        play_req.pitch = 0
        resp = self.client_play(play_req)

    def speak(self, sentence_msg):
        req = SpeakRequest()
        req.sentence = sentence_msg.data
        req.character = self.character
        req.language = self.language
        req.voice = self.voice
        req.emotion = self.emotion
        req.blocking_call = False

        self.do_tts(req)

    def speak_srv(self, req):
        self.do_tts(req)
        return ""

if __name__ == "__main__":
    rospy.init_node('text_to_speech')

    tts_module = rospy.get_param('~tts_module')
    pitch = rospy.get_param('~pitch', 0)
    executable = None
    key = None

    if tts_module == "philips":
        executable = rospy.get_param('~philips_executable')
        if not executable:
            rospy.logerr("text_to_speech_philips: missing parameter 'philips_executable'."
                         "Please provide the location of the executable as ROS parameter.")
            exit(1)
        key = rospy.get_param('~key')
        if not key:
            rospy.logerr("text_to_speech_philips: missing parameter 'key'."
                         "Please provide the Philips key as ROS parameter.")
            exit(1)
        # remove new lines from the key
        key = key.rstrip('\n')

    tts = TTS(tts_module, executable, key, pitch)

    rospy.spin()
