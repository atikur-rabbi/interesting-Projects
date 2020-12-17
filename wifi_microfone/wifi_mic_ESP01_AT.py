# You can freely use the code just leave a link to my channel "ElectroHobby"
#https://www.youtube.com/c/ElectroHobby1 
import socket
import sys
import os
import time
import wave
import struct

############################################################################
QUANT_MASK = (0xf)  #/* Quantization field mask. */ 
SEG_MASK   = (0x70) #/* Segment field mask. */
SEG_SHIFT  = (4)    #/* Left shift for segment number. */
SIGN_BIT   = (0x80) #/* Sign bit for a A-law byte. */
sampleRate = 48000.0 # hertz #sampleRate = 44100.0
record_len = 15     # 5 seconds
dc_ofcet = 1900
volume = 10
############################################################################
slash = '\\'
file_record = '48_3.wav'
obj = wave.open(file_record ,'w')
obj.setnchannels(1) # mono
obj.setsampwidth(2) # n of bytes on 1 sample
obj.setframerate(sampleRate)
a2l_i = 0
############################################################################
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
############################################################################   
sock = socket.socket()
#sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind( ("", 1222) )#port na cotorom my budem prinimat soobcheniya
sock.listen(5)
conn, addr = sock.accept()
print("New connection from ",addr)
stream = ''
############################################################################
print("start_record")
while True:
   stream += conn.recv(2048)
   if len(stream)>=sampleRate*record_len:
      print("recordfinish")
      for i in range (int(sampleRate*record_len)):
         a2l_i=alaw2linear(ord(stream[i]))
         #a2l_i = (a2l_i - dc_ofcet)*volume
         a2l_i = (a2l_i) * volume #dc_ofcet in microcontroller
         wav_data = struct.pack('<h', a2l_i)
         obj.writeframesraw(wav_data)
      break
obj.close()
sock.close()
location = os.getcwd()
os.startfile(location + slash + file_record)
