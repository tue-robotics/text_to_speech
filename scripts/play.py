#! /usr/bin/python

from text_to_speech.srv import Play, PlayRequest

import sys
import rospy


def main():
    rospy.init_node('play_audio')

    if len(sys.argv) != 3:
        print "Usage: play.py FILENAME SERVICE_NAME"
        return 1

    filename = sys.argv[1]
    service_name = sys.argv[2]

    rospy.wait_for_service(service_name)

    try:
        srv_play = rospy.ServiceProxy(service_name, Play)

        req = PlayRequest()
        req.audio_type = "wav"
        req.audio_data = open(filename, "rb").read()
        req.blocking_call = False

        req.pitch = 0

        resp = srv_play(req)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":
    sys.exit(main())
