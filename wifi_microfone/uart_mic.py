# -*- coding: utf-8 -*- 
#!/usr/bin/python

import serial
import struct
import sys
import time
import wave
volume = 15
dc_ofcet = 1900
record_len = 5      # 5 seconds
sampleRate = 8000.0 # hertz #sampleRate = 44100.0 
QUANT_MASK = (0xf)  #/* Quantization field mask. */ 
SEG_MASK   = (0x70) #/* Segment field mask. */
SEG_SHIFT  = (4)    #/* Left shift for segment number. */
SIGN_BIT   = (0x80) #/* Sign bit for a A-law byte. */
####################################################################
s = serial.Serial()
s.port = "COM16"
s.baudrate = 921600
s.bytesize = serial.EIGHTBITS
s.parity = serial.PARITY_NONE
s.stopbits = serial.STOPBITS_TWO
s.timeout = 1
s.xonxoff = False
s.rtscts = False
s.dsrdtr = False
s.open()
####################################################################
obj = wave.open('sound.wav','w')#файл создастся если его нет
obj.setnchannels(1) # mono
obj.setsampwidth(2)
obj.setframerate(sampleRate)
####################################################################
#alaw2linear() - Convert an A-law value to 16-bit linear PCM
def  alaw2linear(a_val):		
    t = 0;      
    seg = 0;    
    a_val ^= 0x55;
    t = (a_val & QUANT_MASK) << 4;
    seg = (a_val & SEG_MASK) >> SEG_SHIFT;#(unsigned)!!!!!!!!!!????
    if seg == 0:
        t += 8;
    elif seg == 1:
        t += 0x108;
    else:
        t += 0x108;
        t <<= seg - 1;
    if(a_val & SIGN_BIT):return t
    else: return -t
####################################################################    

stream = ''
ch = ""
a2l_i = 0 
while True:
   while len(ch)==0:
      ch = s.read(1)
   else:
      for i in range(int(sampleRate*record_len)):
          stream += s.read(1)
      try:
         for i in range (int(sampleRate*record_len)):
            a2l_i=alaw2linear(ord(stream[i]))
            a2l_i = (a2l_i - dc_ofcet)*volume
            wav_data = struct.pack('<h', a2l_i)#
            obj.writeframesraw(wav_data)
         break

      except Exception as e:
         print(e)
obj.close()
