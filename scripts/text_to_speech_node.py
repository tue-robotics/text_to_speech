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
from text_to_speech.srv import GetStatus, Speak, SpeakRequest


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
        self.srv_get_status   = rospy.Service('~get_status',   GetStatus, self.get_status_srv)
        self.srv_speak        = rospy.Service('~speak',        Speak,     self.speak_srv)
        self.srv_clear_buffer = rospy.Service('~clear_buffer', Empty,     self.clear_buffer_srv)

        # buffering tts requests
        self.buffer = []
        self.is_playing = False

        # the current active module (philips or festival)
        self.tts_module = tts_module

    def do_tts(self, text, character, language, voice, emotion):
        rospy.loginfo('TTS: "' + bcolors.OKBLUE + text + bcolors.ENDC + '"')
        rospy.logdebug(
            "TTS: '{0}' (module: '{1}', character: '{2}', language: '{3}', voice: '{4}', emotion: '{5}')"
            .format(text, self.tts_module, character, language, voice, emotion)
        )

        self.is_playing = True

        if self.tts_module == "philips":
            self.do_tts_philips(text, character, language, voice, emotion)
        else:
            self.do_tts_festival(text, character, language, voice, emotion)

        self.is_playing = False

    def do_tts_philips(self, text, character, language, voice, emotion):
        tts_file = file("/tmp/temp_speech_text.txt", "w")

        tts_file.write("¬<" + character + ">" + '\n') # Add character
        tts_file.write("¬<speaker=" + language + "_" + voice + ">" + '\n') # Add language + voice
        tts_file.write("¬<" + emotion + ">" + text)
        tts_file.close()
        rospy.logdebug("File created: /tmp/temp_speech_text.txt")

        os.system("iconv -f utf8 -t iso-8859-15 </tmp/temp_speech_text.txt >/tmp/temp_speech_text2.txt")
        rospy.logdebug("File converted: /tmp/temp_speech_text2.txt")

        filename = "/tmp/speech.wav"

        command = self.executable + " -i /tmp/temp_speech_text2.txt -k " + self.key + " -o {0}".format(filename) + " > /dev/null 2>&1"

        os.system(command)

        rospy.logdebug(".wav file created: {0}".format(filename))

        # Play the file, gst-launch-0.10 is 0.4 seconds faster than aplay
        # ret = os.system("play {0} pitch {1}".format(filename, self.pitch))
        ret = os.system("play %s > /dev/null 2>&1" % filename)
        # ret = os.system("gst-launch-0.10 filesrc location='{0}' ! wavparse ! audioconvert ! audioresample ! autoaudiosink".format(filename))
        rospy.logdebug(".wav file played, removing temporary files")

        try:
            os.remove("/tmp/temp_speech_text.txt")
            os.remove("/tmp/temp_speech_text2.txt")
            # os.remove(filename)
            rospy.logdebug("All temporary files removed, returning call")
        except OSError, ose:
            rospy.logerr("A file could not be removed: {0}".format(ose))

        try:
            save_filename = '/tmp/'+ ''.join(ch for ch in text if ch.isalnum()) + '.wav'
            rospy.logdebug("Saving speech to {0}".format(save_filename))
            os.system("mv {0} {1}".format(filename, save_filename))
        except Exception, e:
            rospy.logerr("Could not save speech to a new file: '{0}'".format(e, filename, save_filename))

        return True

    def do_tts_festival(self, text, character, language, voice, emotion):
        command = "echo \"" + text + "\" | text2wave -o /tmp/festival.wav; play /tmp/festival.wav pitch " + str(self.pitch)
        rospy.logdebug(command)

        p = Popen(command, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p.communicate()
        rc = p.returncode

        if stdout:
            rospy.logwarn(stdout)
        rospy.logdebug(stderr)

    def step(self):
        if not self.buffer or self.is_playing:
            return

        # Get first request from the buffer
        req = self.buffer[0]

        # text to speech (blocking)
        self.do_tts(req.sentence, req.character, req.language, req.voice, req.emotion)

        # If the buffer was not emptied in the meantime, pop the first request
        if self.buffer:
            self.buffer.pop(0)

    def speak(self, sentence_msg):
        req = SpeakRequest()
        req.sentence = sentence_msg.data
        req.character = self.character
        req.language = self.language
        req.voice = self.voice
        req.emotion = self.emotion

        self.buffer += [req]

    def speak_old_srv(self, req):
        self.do_tts(req.sentence, req.character, req.language, req.voice, req.emotion)
        return True

    def get_status_srv(self, req):
        return [[str(req.sentence) for req in self.buffer]]

    def speak_srv(self, req):
        self.buffer += [req]

        if req.blocking_call:
            while not rospy.is_shutdown() and self.buffer:
                self.step()
                rospy.sleep(0.1)

        return ""

    def clear_buffer_srv(self, req):
        self.buffer = []
        return []

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

    while not rospy.is_shutdown():
        tts.step()
        rospy.sleep(0.1)
