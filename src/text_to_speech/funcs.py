from gtts import gTTS, gTTSError
import re
import rospy
from subprocess import Popen, PIPE


def _command(command, label=""):
    if not label:
        label = command.split()[0]
    rospy.logdebug(command)
    p = Popen(command, shell=True, stdout=PIPE, stderr=PIPE)
    stdout, stderr = p.communicate()
    rc = p.returncode

    if stdout:
        rospy.logwarn(stdout)
    if rc != 0:
        msg = "Non-zero return code {}({}): {}".format(label, rc, stderr or "")
        rospy.logerr(msg)
    elif stderr:
        rospy.logerr("{}: {}".format(label, stderr))

    return rc


def play_file(filename):
    command = "gst-launch-1.0 filesrc location='{0}' ! mpegaudioparse ! mpg123audiodec ! audioconvert ! audioresample ! alsasink".format(
        filename)
    return _command(command, "gst-launch")


def festival_to_file(text, output_filename=None):
    """

    :param text:
    :param output_filename: extension should be 'wav'
    :return:
    """
    if output_filename is None:
        output_filename = "/tmp/festival_tts.wav"
    command = "echo \"{}\" | text2wave -o {}".format(text, output_filename)
    rc = _command(command, "Festival TTS")
    if rc:
        raise RuntimeError("Could not generate audio file with Festival")
    return output_filename


def google_tts_to_file(text, language="en", slow=False, output_filename=None):
    """

    :param text:
    :param language:
    :param slow:
    :param output_filename: extension should be `mp3`
    :return: output_filename
    """
    if output_filename is None:
        output_filename = "/tmp/google_tts.mp3"

    try:
        tts = gTTS(text, lang=language, slow=slow)
        tts.save(output_filename)
    except AssertionError:
        rospy.logwarn("gTTS preprocessing of '{}' resulted in an empty string".format(text))
        return ""
    except (ValueError, RuntimeError, gTTSError) as e:
        msg = "gTTS couldn't generate audio file '{}', because: {}".format(output_filename, e)
        rospy.logerr(msg)
        raise RuntimeError(msg)
    return output_filename


def philips_tts_to_file(executable, key, text, language="us", character="default", emotion="neutral", voice="kyle", output_filename=None):
    if output_filename is None:
        save_name = re.sub(r'(\W+| )', '', text)
        output_filename = "/tmp/philips_tts_{}.wav".format(save_name)
    text_file = "¬<" + character + ">" + '\n'  # Add character
    text_file += "¬<speaker=" + language + "_" + voice + ">" + '\n'  # Add language + voice
    text_file += "¬<" + emotion + ">" + text

    tmp_text_file = "/tmp/temp_speech_text.txt"

    with open(tmp_text_file, "w") as tts_file:
        tts_file.write(text.encode("utf8"))

    rospy.loginfo("File created: %s" % tmp_text_file)

    command = executable + " -i {0} -k ".format(tmp_text_file) + key + " -o {0}".format(output_filename)
    rc = _command(command, "Philips TTS")
    if rc:
        raise RuntimeError("Could not generate audio file with Philips")
    return output_filename
