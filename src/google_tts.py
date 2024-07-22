#! /usr/bin/env python
'''Adapted from http://glowingpython.blogspot.nl/2012/11/text-to-speech-with-correct-intonation.html'''

import roslib; roslib.load_manifest('text_to_speech_google')
import rospy
from text_to_speech.srv import Speak
import urllib2
from std_msgs.msg import String
import os, time


def parseText(text):
 """ returns a list of sentences with less than 100 caracters """
 toSay = []
 punct = [',',':',';','.','?','!'] # punctuation
 words = text.split(' ')
 sentence = ''
 for w in words:
  if w[len(w)-1] in punct: # encountered a punctuation mark
   if (len(sentence)+len(w)+1 < 100): # is there enough space?
    sentence += ' '+w # add the word
    toSay.append(sentence.strip()) # save the sentence
   else:
    toSay.append(sentence.strip()) # save the sentence
    toSay.append(w.strip()) # save the word as a sentence
   sentence = '' # start another sentence
  else:
   if (len(sentence)+len(w)+1 < 100):
    sentence += ' '+w # add the word
   else:
    toSay.append(sentence.strip()) # save the sentence
    sentence = w # start a new sentence
 if len(sentence) > 0:
  toSay.append(sentence.strip())
 return toSay

def ping_google():
    failure_codes = [512]
    result = os.system("fping -c1 -t100 www.google.com") #sudo apt-get install fping, -t indicates max millisecond to use, -c1 = 1 packet
    print result
    if int(result) in failure_codes:
        return False
    else:
        return True

def festival_tts(text):
    os.system("echo {0} | festival --tts".format(text))

def tts(text, language='en'):
    if ping_google():
        try:
            google_tts(text, language)
        except Exception as ex:
            rospy.logerr(ex)
            festival_tts(text)
    else:
        rospy.logwarn("No ping to www.google.com, using Festival instead. No language support")
        festival_tts(text)

def google_tts(text, language='en'):
    #text = 'Think of color, pitch, loudness, heaviness, and hotness. Each is the topic of a branch of physics.'

    print text
    if len(text) >= 100:
        toSay = parseText(text)
    else:
        toSay = [text]

    google_translate_url = 'http://translate.google.com/translate_tts'
    opener = urllib2.build_opener()
    opener.addheaders = [('User-agent', 'Mozilla/4.0 (compatible; MSIE 6.0; Windows NT 5.0)')]

    files = []

    generation_start = time.time()

    for i,sentence in enumerate(toSay):
        print i,len(sentence), sentence
        response = opener.open(google_translate_url+'?q='+sentence.replace(' ','%20')+'&tl={0}'.format(language))
        if not response:
            raise Exception()

        filename = str(i)+'speech_google.mp3'
        ofp = open(filename,'wb')
        ofp.write(response.read())
        ofp.close()
        files += [filename]

    generation_end = time.time()
    rospy.loginfo("Downloading audio took {0}s".format(generation_end - generation_start))

    filenames = " ".join(files)
    play_start = time.time()

    #os.system('mpg123 -q '+filenames)
    #os.system('player '+filenames)
    for filename in files:
        #gstreamer command taken from:
        #http://www.cin.ufpe.br/~cinlug/wiki/index.php/Introducing_GStreamer#Play_a_mp3_file
        os.system("gst-launch-0.10 filesrc location='{0}' ! mad ! audioconvert ! audioresample ! alsasink".format(filename))
    play_end = time.time()
    rospy.loginfo("Playing took {0}s".format(play_end - play_start))

def speak_up(str_msg):
    try:
        tts(str_msg.data)
        return True
    except Exception as e:
        rospy.logerr(e)
        return False
    return False

def speak_up_advanced(req):
    try:
        if req.language == "us":
            req.language = "en" #"us" is not a language

        rospy.loginfo("Perfoming TTS in language {0.language} for sentence: '{0.sentence}'".format(req))
        tts(req.sentence, req.language)
        return "True"
    except Exception as e:
        rospy.logerr(e)
        return "False"
    return "False"

if __name__ == "__main__":
    rospy.init_node('text_to_speech_google')
    language = rospy.get_param("/text_to_speech/language",   "us")

    subscriber  = rospy.Subscriber("/text_to_speech/input", String, speak_up)

    rospy.Service('/text_to_speech/speak', Speak, speak_up_advanced)

    rospy.loginfo("Listener /text_to_speech/input and Service amigo_speakup_advanced started")
    rospy.logwarn("Google TTS does not take voice and emotion into account")

    rospy.spin()
