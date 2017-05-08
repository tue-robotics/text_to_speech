#!/usr/bin/python
import os
TTS_EXE_FILE = "../exec/ptts_v911.exe"
filename = "/tmp/speech.wav"
os.system("rm {0}".format(filename))
#command = TTS_EXE_FILE+" -i ../mbrola_test.txt -k y2ntwg43ctyg7rz4bv9ljxsdxqnv02p0s2ky--5klcxy94vh23ttzt4q317q -o {0}".format(filename)
command = TTS_EXE_FILE + " -i ../singing_US.txt -k xcyst4l363x6c5j40tzz-v4d6kgt0tr0c7hj1l3lq-cmsn39tskyz9cvwv4z -o {0}".format(filename)
os.system(command)
#os.system("gst-launch-0.10 filesrc location='{0}' ! wavparse ! audioconvert ! audioresample ! osssink".format(filename))
os.system("aplay {0}".format(filename))
