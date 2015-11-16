#!/usr/bin/env python
# coding: utf-8

'''
This node listens to a service call and a topic for text to speech
requests. These will be processed by the festival or the philips tts module.
'''

import rospy
import os

from subprocess import Popen, PIPE
from std_msgs.msg import String
from std_srvs.srv import Empty
from text_to_speech.srv import GetStatus, Speak, SpeakRequest, Play, PlayRequest


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

        self.character = rospy.get_param("~character",  "Default")
        self.language  = rospy.get_param("~language",   "us")
        self.voice     = rospy.get_param("~voice",      "kyle")
        self.emotion   = rospy.get_param("~emotion",    "Neutral")

        # topics
        self.sub_speak = rospy.Subscriber("~input", String, self.speak)

        # services
        self.srv_speak        = rospy.Service('~speak',        Speak,     self.speak_srv)

        # clients
        self.client_play = rospy.ServiceProxy('play', Play)

        # the current active module (philips or festival)
        self.tts_module = tts_module



    def do_tts(self, req):
        rospy.loginfo('TTS: "' + bcolors.OKBLUE + req.sentence + bcolors.ENDC + '"')
        rospy.logdebug(
            "TTS: '{0}' (module: '{1}', character: '{2}', language: '{3}', voice: '{4}', emotion: '{5}')"
            .format(req.sentence, self.tts_module, req.character, req.language, req.voice, req.emotion)
        )

        if self.tts_module == "philips":
            self.do_tts_philips(req)
        else:
            self.do_tts_festival(req)

    def do_tts_philips(self, req):
        tts_file = file("/tmp/temp_speech_text.txt", "w")

        tts_file.write("¬<" + req.character + ">" + '\n') # Add character
        tts_file.write("¬<speaker=" + req.language + "_" + req.voice + ">" + '\n') # Add language + voice
        tts_file.write("¬<" + req.emotion + ">" + req.sentence)
        tts_file.close()
        rospy.logdebug("File created: /tmp/temp_speech_text.txt")

        os.system("iconv -f utf8 -t iso-8859-15 </tmp/temp_speech_text.txt >/tmp/temp_speech_text2.txt")
        rospy.logdebug("File converted: /tmp/temp_speech_text2.txt")

        filename = "/tmp/speech.wav"

        command = self.executable + " -i /tmp/temp_speech_text2.txt -k " + self.key + " -o {0}".format(filename) + " > /dev/null 2>&1"

        os.system(command)

        rospy.logdebug(".wav file created: {0}".format(filename))

        # Play sound
        play_req = PlayRequest()
        play_req.audio_data = open(filename, "rb").read()
        play_req.audio_type = "wav"
        play_req.blocking_call = req.blocking_call
        play_req.pitch = 0
        resp = self.client_play(play_req)

        if resp.error_msg:
            rospy.logerr("Could not play sound: %s", resp.error_msg)

        rospy.logdebug(".wav file played, removing temporary files")

        try:
            os.remove("/tmp/temp_speech_text.txt")
            os.remove("/tmp/temp_speech_text2.txt")
            # os.remove(filename)
            rospy.logdebug("All temporary files removed, returning call")
        except OSError, ose:
            rospy.logerr("A file could not be removed: {0}".format(ose))

        try:
            save_filename = '/tmp/'+ ''.join(ch for ch in req.sentence if ch.isalnum()) + '.wav'
            rospy.logdebug("Saving speech to {0}".format(save_filename))
            os.system("mv {0} {1}".format(filename, save_filename))
        except Exception, e:
            rospy.logerr("Could not save speech to a new file: '{0}'".format(e, filename, save_filename))

        return True

    def do_tts_festival(self, req):
        filename = "/tmp/festival.wav"

        command = "echo \"%s\" | text2wave -o %s" % (req.sentence, filename)
        rospy.logdebug(command)

        p = Popen(command, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p.communicate()
        rc = p.returncode

        if stdout:
            rospy.logwarn(stdout)
        rospy.logdebug(stderr)

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
            rospy.logerr("text_to_speech_philips: missing parameter 'philips_executable'. Please provide the location of the executable as ROS parameter.")
            exit(-1)
        key = rospy.get_param('~key')
        if not key:
            rospy.logerr("text_to_speech_philips: missing parameter 'key'. Please provide the Philips key as ROS parameter.")
            exit(-1)
        # remove new lines from the key
        key = key.rstrip('\n')

    tts = TTS(tts_module, executable, key, pitch)

    rospy.spin()
