#! /usr/bin/env python

from pyaudio import PyAudio, paInt16
import numpy as np
import wave
import os
import time

#timecounter = 5
#for i in range(5):
#    timecounter = 5 - i
#    print('recoring will start in', timecounter, 'seconds')
#    time.sleep(1)


class Recoder:
    NUM_SAMPLES = 2000
    SAMPLING_RATE = 16000
    LEVEL = 500
    COUNT_NUM = 20
    SAVE_LENGTH = 8
    Voice_String = []

    def savewav(self, filename):
        wf = wave.open(filename, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(self.SAMPLING_RATE)
        wf.writeframes(np.array(self.Voice_String).tostring())
        wf.close()

    def recoder(self):
        pa = PyAudio()
        stream = pa.open(format=paInt16, channels=1, rate=self.SAMPLING_RATE, input=True,
                         frames_per_buffer=self.NUM_SAMPLES)
        save_count = 0
        save_buffer = []
        while True:
            string_audio_data = stream.read(self.NUM_SAMPLES)
            audio_data = np.fromstring(string_audio_data, dtype=np.short)
            large_sample_count = np.sum(audio_data > self.LEVEL)
            # print(np.max(audio_data))
            if large_sample_count > self.COUNT_NUM:
                save_count = self.SAVE_LENGTH
            else:
                save_count -= 1
            if save_count < 0:
                save_count = 0
            if save_count > 0:
                save_buffer.append(string_audio_data)
            else:
                if len(save_buffer) > 0:
                    self.Voice_String = save_buffer
                    save_buffer = []
                    print("Recode a piece of voice successfully!")
                    return True
        	else:
            	    return False


if __name__ == "__main__":
    r = recoder()
    r.recoder()
