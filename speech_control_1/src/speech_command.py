#!/usr/bin/env python

import rospy
import os
from pocketsphinx import LiveSpeech, get_model_path
from std_msgs.msg import String

model_path = get_model_path()

speech = LiveSpeech(
    verbose=False,
    sampling_rate=16000,
    buffer_size=2048,
    no_search=False,
    full_utt=False,
    hmm=os.path.join(model_path, 'en-us'),
    lm=os.path.join(model_path, 'en-us.lm.bin'),
    dic=os.path.join(model_path, 'cmudict-en-us.dict')
)

global command_speech

def talker():
    rospy.init_node('Object_Tracking')
    pub = rospy.Publisher('/jetbot_motors/cmd_str', String, queue_size=100)
    while not rospy.is_shutdown():
        for phrase in speech:
            command_speech = str(phrase)
            print(command_speech)
            if command_speech == 'go':
                pub.publish ("backward")
            elif command_speech == 'back':
                pub.publish ("forward")
            elif command_speech == 'stop':
                pub.publish ("stop")
            elif command_speech == 'left':
                pub.publish ("right")
            elif command_speech == 'right':
                pub.publish ("left")

    rate.sleep()
        
   
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
