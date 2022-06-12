# Speech-controlled Jetbot
A simple implementation of speech control for Jetbot

## Project Description
The project aims to control Jetbot wirelessly to do simple maneuver (i.e. going forward, going backward, rotating left, rotating right) with voice command (e.g. "go" = forward, "back" = backward, "left" = left, "right" = right), can be easily modified for any kind of robot platform. 

The project uses: 
* [ROS Framework](http://wiki.ros.org/) (on Ubuntu) to alleviate concurrency management and module communication.
* [Jetbot](https://jetbot.org/master/), which is a differential wheeled robot and its ros package [jetbot_ros](https://github.com/dusty-nv/jetbot_ros) for motor control.
* [Speech Recognition](https://github.com/Uberi/speech_recognition#readme) module with [Pocketsphinx Python](https://github.com/bambocher/pocketsphinx-python) library for OFFLINE recognition.

## Installation
* Install Python.
* [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) (any version).
* At /usr/home/"name" ("name" is arbitrary), create a ROS workspace. On terminal: 
   ```sh
   mkdir -p ~/catkin_ws/src
   ```
* Go to the created ROS workspace, clone the [jetbot_ros](https://github.com/dusty-nv/jetbot_ros) repo (choose the correct path for your ROS) and build it. On terminal: 
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/dusty-nv/jetbot_ros -b melodic
   cd ..
   catkin_make
   ```
* If microphone is used, install PyAudio. PortAudio is be needed.
   ```sh
   sudo apt-get install portaudio19-dev 
   pip install PyAudio
   ```
* Install [Speech Recognition](https://github.com/Uberi/speech_recognition#readme) module and [Pocketsphinx Python](https://github.com/bambocher/pocketsphinx-python). SWIG is needed.
   ```sh
   pip install SpeechRecognition
   sudo apt update
   sudo apt install swig
   python -m pip install --upgrade pip setuptools wheel
   pip install --upgrade pocketsphinx
   ```
## Project Reconstruction
* Create a Python file with an arbitrary name (e.g. speech_test.py), copy & paste the first example of [Pocketsphinx Python](https://github.com/bambocher/pocketsphinx-python).
   ```sh
   import os
   from pocketsphinx import LiveSpeech, get_model_path

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

   for phrase in speech:
       print(phrase)
   ```
* Save and test speech_test.py. On terminal: 
   ```sh
   python speech_test.py
   ```
* Speak to the microphone and observe the result. Make sure the words used to control the robot is detected easily (e.g. "go","back","left","right"). If not, changing the keywords or better pronunciation are advised. 
* Create a new ROS package. On terminal: 
   ```sh
   cd ~/catkin_ws/src
   catkin_create_pkg speech_control_1
   cd speech_command
   mkdir src
   cd src
   ```
* Create the source code for the created ROS package with an arbitrary name (e.g. speech_command.py). Combining the [Publisher-Subscriber in Python tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) with speech_test.py and ROS topic `/jetbot_motors/cmd_str`, which means we simply need to import and init a ros(py) entity to speech_test.py, open a publisher that publishes string commands to `/jetbot_motors/cmd_str`. 
   ```sh
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
   ```
