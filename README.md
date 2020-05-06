ReTiCo is a python framework to enable real time incremental conversational spoken dialogue systems.
The architecture is based on the paper of Schlangen and Skantze "A General, Abstract Model of Incremental Dialogue Processing" (2011).

## Framework

The framework provides base classes for modules and *incremental units* that can be used to write asynchronous data processing pipelines.

## ReTiCo Builder

The ReTiCo Builder provides a graphical user interface for building networks with the modules available.

## Installing

### Requirements

Generally you need the following for running ReTiCo:

 - `Python 3`
 - `portaudio`
 - `pyaudio`
 - `flexx`

 Optionally you can use additional modules if you install:

 - `google-cloud-speech`
 - `rasa-nlu`
 - `incremental-rasa-nlu`
 - `pyopendial`


#### Portaudio

Because ReTiCo is handling all sorts of audio streams, you need to have `portaudio` installed.

On **Linux** this can be done via your favorite package-manager. For example:

```
$ sudo apt-get install portaudio19-dev python-pyaudio python3-pyaudio
```

On **MacOS** you can install it via `brew`:

```
$ brew install portaudio
```

On **Windows** you need to google.

#### PyAudio

For this to work you need to install portaudio on your system first!

```
$ pip install pyaudio
```

### Flexx

To run the retico_builder frontend you need at least flexx 0.4.2. For this you can clone the repository from github and install it from there.

```
$ pip install https://github.com/flexxui/flexx/archive/master.zip
```

## Optional stuff

Some third party libraries are not included into ReTiCo and have to be installed manually. If you try to use those IncrementalModules before you installed the library you will get an error.

In the ReTiCo Builder, the modules won't appear in the list of available modules until you install the third pary library.

### ZeroMQ

For easy interop between distributed modules, or between retico and other programming languages, you can use ZeroMQ. 

For ubuntu:

```
sudo apt install python-zmq
```

### Google Cloud Speech

To use the Automatic Speech Recognition module utilizing google cloud speech, you need to install the thrid party package first.

For this you may follow the first two steps of [this tutorial](https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries#client-libraries-install-python).

Important is, that you create your **Google Application Credentials json file** and save the path to that file into the global variable `GOOGLE_APPLICATION_CREDENTIALS` (look for the "*Before you begin*" section on the tutorial page).

If this is setup and you've run

```
$ pip install google-cloud-speech
```

you are all set for using GoogleASR!

### Rasa NLU

For installing Rasa NLU, just install it via:

```
$ pip install rasa_nlu
```

### Incremental Rasa NLU

If you want to use an incremental version of Rasa NLU, use

```
git clone https://bitbucket.org/bsu-slim/incremental-rasa-nlu
```

Then open the incremental-rasa-nlu folder and run `pip install -r requirements.txt`

Finally, set the `RASA` environment variable to the incremental-rasa-nlu folder.

### PyOpenDial

For Installing PyOpenDial, use

```
git clone https://bitbucket.org/bsu-slim/pyopendial
```

Then open the pyopendial folder and run `pip install -r requirements.txt`

Finally, set the `PYOD` environment variable to the pyopendial folder. 

### Object Detection 

For using the google object detection module, you'll need to follow the instructions here:

```
https://github.com/tensorflow/models/tree/master/research/object_detection
```

You will then need to set the TF_RESEARCH environment variable to the models/research folder, 
and the TF_SLIM environment variable to the models/research/slim folder. 

### eSpeak

For using eSpeak module, you will need to install on your system

```
$ pip install py-espeak-ng
```

Then on **Linux** run the following command

```
$ sudo apt install espeak-ng
```


### Misty robot

There are two ways to use Misty camera:

* Using Retico module, make sure Retico is installed and make sure you are connected to the same network that misty is connected to, It can be subscribed to like any other Retico module, you will need to pass in misty’s IP, and optionally width and height values for the picture size if they are needed to be different from the default.

To run the test:

```
$ python test_misty_camera.py
```

* Without using Retico and that can be achieved by changing the IP to misty’s correct IP, Misty will take a picture and translate it to base64, as well as save the image to a file. 

```
$ python mistypicture.py
```

To run with basic random movements 

```
$ python mistyidle.py
```

### opencv face detection


The module will run OpenCV's face detection algorithm and will return (x,y) coordinates for the bounding boxes of each face as well as the end coordinates, before running it make sure you install Retico, and your computer has a camera, finally install NumPy using the command bollow:

```
$ pip install numpy
```

In order to test the module, in the root retico folder there is a test_face_detection.py script that will run the module on your computer using your camera and you will be able to see the output printed on the screen.

To run the test:

```
$ python test_face_detection.py 
```

### Cozmo Robot

If you want to use Cozmo, follow the installation instructions found on the [Anki Cozmo Website](https://developer.anki.com/blog/learn/tutorial/getting-started-with-the-cozmo-sdk/index.html).

However, you'll need to use our cozmo sdk fork:

```
https://bitbucket.org/bsu-slim/cozmo-python-sdk
```

Then you'll need to set the COZMO environment variable to your cozmo-python-sdk/src folder.


### ROS
Retico modules can make use of interop communications using ROS

To use ROS, you need to follow installation steps of [this installation](http://wiki.ros.org/lunar/Installation/Ubuntu).

For an exemple about ROS, click on [this link](https://github.com/Microsoft/psi/tree/master/Samples/RosTurtleSample).

ros can be used in conjunction with retico utilizing the ros python client library

```bash
$ pip install rospy
```

You will have to be able to use ros tools in order to run the examples which can be found
in ./examples/test_audio_ros.py as well as ./examples/Turtlesim_Teleop.py


## Installing Retico

To setup retico you can just run

```
$ python setup.py install
```

## Using Retico

### In Python

In Python you can now import the ReTiCo modules, connect them and run your network.

```python
m1 = MicrophoneModule(5000)
m2 = StreamingSpeakerModule(5000)

m1.subscribe(m2)

m1.run()
m2.run()

input()

m1.stop()
m2.stop()
```

This example will create a simple network consisting of a MicrophoneModule that gives its output to the StreamingSpeakerModule, which just outputs the incoming data to the speaker.


### With the GUI

If you have installed flexx, you can simply run

```
$ python -m retico_builder.builder
```
