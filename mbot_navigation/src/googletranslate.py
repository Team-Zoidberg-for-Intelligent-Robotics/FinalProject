#! /usr/bin/env python
# -*- coding: utf-8 -*-

import speech_recognition as sr
import re
import copy
import record
import time

class Translate:
    def __init__(self):
        self.room_list = ['roomone', 'room1',
                          'roomtwo', 'room2',
                          'roomthree', 'room3',
                          'roomfour', 'room4',
                          'roomfive', 'room5',
                          'roomsix', 'room6',
                          'please say again']
        self.room_number = {'roomone': 0, 'room1': 0,
                            'roomtwo': 1, 'room2': 1,
                            'roomthree': 2, 'room3': 2,
                            'roomfour': 3, 'room4': 3,
                            'roomfive': 4, 'room5': 4,
                            'roomsix': 5, 'room6': 5}
        self.r = sr.Recognizer()
        self.voice = None
        self.T = None
        self.string_result = None
        self.result = None
        self.source = None
        self.audio = None
        self.realRoom_number = None
	self.record = record.Recoder()
	self.start = True
	self.repeat = False

    def translate(self):
        self.voice = sr.AudioFile('/home/zm/Desktop/test.wav')
        with self.voice as self.source:
            self.r.adjust_for_ambient_noise(self.source, duration=1.)
            self.audio = self.r.record(self.source)
        self.T = self.r.recognize_google(self.audio, show_all=True)
        self.string_result = str(self.T)[1:-1]
        tmpResult = copy.deepcopy(self.string_result)
        self.result = re.sub("[\s+\.\!\/_,$%^*(+\"\'\{\}\[\]\:]", "", tmpResult)
	return self.result

    def match(self):
        result = self.translate()
        if result is None or result == '':
            self.rerecord()
        else:
            for room in range(len(self.room_list)):
		print("10000",self.result)
                if self.room_list[room] in self.result and self.result is not None:
                    print("Do you need me to navigate to", self.room_list[room], "for you")
		    print("20000",self.result)
   		    while self.start:
                        r = record.Recoder()
                        r.recoder()
                        r.savewav("/home/zm/Desktop/test.wav")
                        opreate = self.translate()
                    	if "yes" in opreate:
                            self.realRoom_number = self.room_list[room]
                            self.start = False
                        elif "no" in opreate:
                            print("Please repeat which room you are going to")
                            self.repeat = True
			    self.start = False
			    self.result = None
			    break
		    print(self.result)
		    break
		elif self.repeat:
		    break
                else:
		    if room+1 == len(self.room_list):
			print("I do not know what you want to do, please speak again")
                    	self.rerecord()
	if self.repeat:
	    self.repeat = False
	    self.start = True
	    self.rerecord()	

    def getRealRoomNumber(self):
        if not self.realRoom_number is None:
            if self.realRoom_number in self.room_number:
                return self.room_number[self.realRoom_number]
        else:
            return None


    def rerecord(self):
	self.record.recoder()
    	self.record.savewav("/home/zm/Desktop/test.wav")
	self.match()


if __name__ == '__main__':
    t = Translate()
    t.match()
    print(t.getRealRoomNumber())

