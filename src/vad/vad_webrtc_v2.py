#!/usr/bin/python

'''
Requirements:
+ pyaudio - `pip install pyaudio`
+ py-webrtcvad - `pip install webrtcvad`
'''
import rospy
from std_msgs.msg import Bool

import webrtcvad
import collections
import sys
import signal
import pyaudio
import contextlib

from array import array
from struct import pack
import wave
import time

from vad_webrtc import WebRTC
from vad_listener import VADListener

class VADWebRTCV2(WebRTC) :

    def __init__(self, observer, chunk_duration_ms = 30, padding_duration_ms = 1500, 
        format = pyaudio.paInt16, channels = 1, rate = 16000) :


        self.format = pyaudio.paInt16
        self.channels = channels
        self.rate = rate
        self.chunk = int(self.rate * chunk_duration_ms / 1000)
        self.starting = False
        
        chunk_bytes = self.chunk * 2  # 16bit = 2 bytes, PCM
        self.num_padding_chunks = int(padding_duration_ms / chunk_duration_ms)
        # num_windows_chunk = int(240 / chunk_duration_ms)
        self.num_windows_chunk = int(400 / chunk_duration_ms)  # 400 ms/ 30ms  ge
        self.num_windows_chunk_END = self.num_windows_chunk * 2

        START_OFFSET = int(self.num_windows_chunk * chunk_duration_ms * 0.5 * self.rate)

        self.vad = webrtcvad.Vad(1)
        signal.signal(signal.SIGINT, self.handle_int)
        #rospy.on_shutdown(self.handle_int)

        WebRTC.__init__(self, observer)

        

    def process_chunk(self) :

        

        got_a_sentence = False
        leave = False

        while not leave and self.running :
            if self.speaking:
                ring_buffer.clear()
                break

            ring_buffer = collections.deque(maxlen=self.num_padding_chunks)
            triggered = False
            voiced_frames = []
            ring_buffer_flags = [0] * self.num_windows_chunk
            ring_buffer_index = 0

            ring_buffer_flags_end = [0] * self.num_windows_chunk_END
            ring_buffer_index_end = 0
            buffer_in = ''
            # WangS
            raw_data = array('h')
            index = 0
            start_point = 0
            #StartTime = time.time()
            print("* recording: ")
            self.stream.start_stream()

            while not got_a_sentence and not leave and self.running :

                if self.speaking:
                    ring_buffer.clear()
                    break

                chunk = self.stream.read(self.chunk)
                # add WangS
                raw_data.extend(array('h', chunk))
                index += self.chunk
                #TimeUse = time.time() - StartTime

                active = self.vad.is_speech(chunk, self.rate)

                sys.stdout.write('1' if active else '_')
                ring_buffer_flags[ring_buffer_index] = 1 if active else 0
                ring_buffer_index += 1
                ring_buffer_index %= self.num_windows_chunk

                ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
                ring_buffer_index_end += 1
                ring_buffer_index_end %= self.num_windows_chunk_END

                # start point detection
                if not triggered:
                    #sys.stdout.write('+')
                    #ring_buffer.append(chunk)
                    num_voiced = sum(ring_buffer_flags)
                    if num_voiced > 0.8 * self.num_windows_chunk:
                        sys.stdout.write(' Open ')
                        triggered = True
                        start_point = index - self.chunk * 20  # start point
                        # voiced_frames.extend(ring_buffer)
                        sys.stdout.write('-')
                        ring_buffer.clear()
                # end point detection
                else:
                    # voiced_frames.append(chunk)
                    #sys.stdout.write('+')
                    ring_buffer.append(chunk)
                    num_unvoiced = self.num_windows_chunk_END - sum(ring_buffer_flags_end)
                    if num_unvoiced > 0.90 * self.num_windows_chunk_END : #or TimeUse > 10
                        sys.stdout.write(' Close ')

                        triggered = False
                        got_a_sentence = True

                sys.stdout.flush()

            sys.stdout.write('\n')
            # data = b''.join(voiced_frames)

            self.stream.stop_stream()
            print("* done recording")
            got_a_sentence = False

            # write to file
            raw_data.reverse()
            for index in range(start_point):
                raw_data.pop()
            raw_data.reverse()
            raw_data = self.normalize(raw_data)

            raw_data = pack('<' + ('h' * len(raw_data)), *raw_data)

            #self.record_to_file('test.wav', raw_data, 2)
            VADListener.notify_observer(self, raw_data, self.rate)

            leave = True

    def handle_int(self, sig, frame):
        self.running = False
        exit(0)


    """"def record_to_file(self, path, data, sample_width):
        "Records from the microphone and outputs the resulting data to 'path'"
        #sample_width, data = record()
        
        with contextlib.closing(wave.open(path, 'wb')) as wf:
        #wf = wave.open(path, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(sample_width)
            wf.setframerate(self.rate)
            wf.writeframes(data)
        #wf.close()"""


    def normalize(self, snd_data):
        "Average the volume out"
        MAXIMUM = 32767  # 16384
        times = float(MAXIMUM) / max(abs(i) for i in snd_data)
        r = array('h')
        for i in snd_data:
            r.append(int(i * times))
        return r


if __name__ == '__main__':
    pass