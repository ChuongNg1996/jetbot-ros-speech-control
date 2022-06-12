# Speech-controlled Jetbot
A simple implementation of speech control for Jetbot

## Project Description
The project aims to control Jetbot wirelessly to do simple maneuver (i.e. going forward, going backward, rotating left, rotating right) with voice command (e.g. "go" = forward, "back" = backward, "left" = left, "right" = right), can be easily modified for any kind of robot platform. 

The project uses: 
* [ROS Framework](http://wiki.ros.org/) (on Ubuntu) to alleviate concurrency management and module communication.
* [Jetbot](https://jetbot.org/master/), which is a differential wheeled robot and its ros package [jetbot_ros](https://github.com/dusty-nv/jetbot_ros) for motor control.
* [Speech Recognition](https://github.com/Uberi/speech_recognition#readme) module with [Pocketsphinx Python](https://github.com/bambocher/pocketsphinx-python) library for OFFLINE recognition.

The project is very short and simple. Thus, a reconstruction will be provided instead of an installation guide.

## Project Reconstruction
* Install Python.
* [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) (any version).
* At /usr/home/"name" ("name" is arbitrary), create a ROS workspace.
   ```sh
   mkdir -p ~/catkin_ws/src
   ```
* Go to the created ROS workspace, clone the [jetbot_ros](https://github.com/dusty-nv/jetbot_ros) repo and build it.
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/dusty-nv/jetbot_ros 
   cd ..
   catkin_make
   ```
* If microphone is used, install PyAudio. PortAudio is be needed
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
