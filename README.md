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

1. Clone and cd in the qt-dialog repository

```bash
        $ git clone https://github.com/AIRobolab-unilu/qt-dialog.git
        $ cd qt-dialog
```


2. It is recommended to use a virtual env

	1. If you do not have pip

	```bash
            $ sudo apt-get install python-pip
    ```

    2. If you do not have virtual env

    ```bash
            $ sudo pip install virtualenv 
    ```

    3. Install pyaudio
    ```bash
            $ sudo apt-get install python-pyaudio
    ```

    4. Create the virtual environment

    ```bash
            $ virtualenv --system-site-packages env
            $ source env/bin/activate
    ```
3. Install the python dependencies

```bash
        $ sudo pip install -r requirements.txt
```

To undo the changes with virtualenv

```bash
        $ deactivate
```

```
WIP
```

### Using

Before using it you need to update your creditentials in your system environment.
This can be done like that :

```bash
export WATSON_USER_STT="{your user}"
export WATSON_PASS_STT="{your password}"

export WATSON_USER_NLP="{your user}" #In most cases the same as WATSON_USER_STT
export WATSON_PASS_NLP="{your password}"

export GOOGLE_USER_STT="{your user}"
export GOOGLE_PASS_STT="{your password}"
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
* [JSGF Tools](https://github.com/syntactic/JSGFTools) -  Used to generate strings from a JSGF grammar

## Authors

* [**Thomas Sauvage**](https://github.com/SauvageThomas)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgments

WIP
Thanks to [PurpleBooth](https://github.com/PurpleBooth) for the template for this readme.

