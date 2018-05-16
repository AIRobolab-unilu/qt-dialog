# qt-dialog

This projects aims to provide dialog under ROS (from Automatic Voice Recognition to Text-To-Speech)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

What things you need to install the software and how to install them

```
ROS kinetic
python2.7
pyaudio
webrtcvad
pocketsphinx
```

### Installing

A step by step series of examples that tell you have to get a development env running

Say what the step will be

```
WIP
```

### Using

Before using it you need to update your creditentials in your system environment.
This can be done like that :

```
export WATSON_USER_STT="{your user}"
export WATSON_PASS_STT="{your password}"

export WATSON_USER_NLP="{your user}" #In most cases the same as WATSON_USER_STT
export WATSON_PASS_NLP="{your password}"
```

### Little demo

WIP

## Running the tests

WIP

## Built With

* [PyAudio](http://people.csail.mit.edu/hubert/pyaudio/) - The software used to record audio input
* [py-webrtcvad](https://github.com/wiseman/py-webrtcvad) -  WebRTC Voice Activity Detector interface for Python
* [svox-tts](https://github.com/ScazLab/svox_tts) -  A ROS wrapper for the svox-pico TTS Engine
* [pyttsx](https://github.com/RapidWareTech/pyttsx) -  Cross-platform Python wrapper for text-to-speech synthesis
* [python-vad](https://github.com/wangshub/python-vad) -  Python code to voice activity detection
* [Watson STT](https://www.ibm.com/watson/services/speech-to-text/) -  Speech-To-Text from Watson - IBM
* [pocketsphinx](https://github.com/Pankaj-Baranwal/pocketsphinx) -  Extension of pocketsphinx in the ROS world

## Authors

* [**Thomas Sauvage**](https://github.com/SauvageThomas)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgments

WIP
Thanks to [PurpleBooth](https://github.com/PurpleBooth) for the template for this readme.

