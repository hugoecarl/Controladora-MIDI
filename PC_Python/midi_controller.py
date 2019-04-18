import pyautogui
import serial
import argparse
import time
import logging
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
import pygame
pygame.mixer.init()



class SerialControllerInterface:
    #Protocolo
    #4 primeiros bytes reservados para o volume, flag em seguida após x para referenciar os samplers dependendo do botao apertado 

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        #self.mapping = MyControllerMap()
        self.incoming = '0'
        self.vol = -1
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        income = []

        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        while self.incoming != b'x':
            self.incoming = self.ser.read()
            if self.incoming != b'x':
                income.append(str(int(self.incoming)))
            logging.debug("Received INCOMING: {}".format(self.incoming))
        
        num = float("".join(income))

        print(num)
        
        print("\n")

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))
        print(data)

        if data == b'a':
            pygame.mixer.music.load("Kick.wav")
            pygame.mixer.music.play()
        elif data == b'b':
            pygame.mixer.music.load("clap.wav")
            pygame.mixer.music.play()
        elif data == b'c':
            pygame.mixer.music.load("snareo.wav")
            pygame.mixer.music.play()

        
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        v = int(round(volume.GetMasterVolumeLevelScalar()*100, 1))
        
        
        if v != self.vol:
            if self.vol != -1:
                self.ser.write(bytes(str(v), 'utf-8'))
                if abs(num/40.95 - v) > 5:
                    print("Ajustando Slider")
                    return

        for i in range(1, 101):
            if num > (i-1)*40.95 and num < (i * 40.95): 
                self.vol = i
                volume.SetMasterVolumeLevelScalar((float(i)/100), None)
                if num <= 32:
                    self.vol = 0
                    volume.SetMasterVolumeLevelScalar(0.0, None)

            
       
        print("Volume", v)

       
        self.incoming = self.ser.read()
        

if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
