from aiohttp import web
import socketio
import asyncio
from gpiozero import CPUTemperature
import serial
import pygame
import os

sio = socketio.AsyncServer()
app = web.Application()
sio.attach(app)
loop = asyncio.get_event_loop()
pygame.display.init()



ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.01
)

async def serLoop():
    line = []
    while True:
        await asyncio.sleep(0.2)
        for c in ser.read():
            line.append(chr(int(c)))
            if chr(int(c)) == '\n':
                strSeg = ''.join(line).replace("\n", "")
                strSeg = strSeg.split(" ")
                if(len(strSeg) > 0):
                    if(strSeg[0] == 'v'):
                        await sio.emit('volt', float(strSeg[1]))
                line = []
                break

async def gameLoop():
    while True:
        try:
            pygame.joystick.init()
            # Check if there is a joystick
            if(pygame.joystick.get_count()) < 1:   
                pygame.joystick.quit()
                await asyncio.sleep(1) 
            else:   
                joystick = pygame.joystick.Joystick(0)
                break
        except pygame.error:
            pygame.joystick.quit()
            await asyncio.sleep(1) 
    while True:       
        try:
            await asyncio.sleep(0.01)
            joystick.init()

            if joystick.get_button(6):
                ser.write(('g 1\n').encode('utf-8'))
            if joystick.get_button(7):
                ser.write(('g -1\n').encode('utf-8'))
            if joystick.get_hat(0)[1] == 1:
                ser.write(('t 0 0 -0.3\n').encode('utf-8'))
            if joystick.get_hat(0)[1] == -1:
                ser.write(('t 0 0 0.3\n').encode('utf-8'))
                
            # Check for events
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:	
                    if joystick.get_button(3):
                        ser.write(('d\n').encode('utf-8'))
                    elif joystick.get_button(4):
                        ser.write(('u\n').encode('utf-8'))
                    elif joystick.get_button(15):
                        ser.write(('a\n').encode('utf-8'))
                    elif joystick.get_button(11):
                        ser.write(('b\n').encode('utf-8'))
                    elif joystick.get_button(0):
                        ser.write(('c\n').encode('utf-8'))
                    elif joystick.get_button(1):
                        ser.write(('o\n').encode('utf-8'))
                    elif joystick.get_button(13):
                        ser.write(('s\n').encode('utf-8'))
                if event.type == pygame.JOYAXISMOTION:                        
                    axis0 = joystick.get_axis(0)
                    axis1 = joystick.get_axis(1)
                    axis2 = joystick.get_axis(2)
                    axis3 = joystick.get_axis(3)
                    axis4 = (joystick.get_axis(4) -joystick.get_axis(5))/2

                    msxString = "{:7.3f}".format(axis0*0.7)
                    msyString = "{:7.3f}".format(-axis1*1.2)
                    msrString = "{:7.3f}".format(axis4*0.8)
                    yawString = "{:7.3f}".format(axis2/5)
                    pitchString = "{:7.3f}".format(axis3/4)
                    #print('m %s %s %s\n'%(msxString,msyString,msrString))
                    if joystick.get_button(6):
                        ser.write(('t %s %s 0\n'%("{:7.3f}".format(-axis0*20), "{:7.3f}".format(-axis1*32))).encode('utf-8'))
                    else:
                        ser.write(('m %s %s %s\n'%(msxString, msyString, msrString)).encode('utf-8'))
                    ser.write(('r 0 %s %s\n'%(pitchString, yawString)).encode('utf-8'))
                # Multiple events are generated for the same axis motion, so break after the first
                break
        except pygame.error:
            await asyncio.sleep(1) 

loop = asyncio.get_event_loop()
loop.create_task(gameLoop())
serialLoop = asyncio.get_event_loop()
serialLoop.create_task(serLoop())

def index(request):
    with open(os.path.dirname(__file__)+'/index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

async def sendTemp():
    while True:
        await asyncio.sleep(3)
        await sio.emit('temp', CPUTemperature().temperature)

loop.create_task(sendTemp())

async def heartbeat():
    while True:
        ser.write(('h\n').encode('utf-8'))
        await asyncio.sleep(1)

loop.create_task(heartbeat())

@sio.on('connect')
def chat_connection(sid, message):
    print('---- connected ----')
    
    
@sio.on('disconnect')
def chat_disconnect(sid):
    print('---- disconnected ----')

@sio.on('mov')
async def position(sid, msx, msy, msr):
    msxString = "{:7.3f}".format(float(msx))
    msyString = "{:7.3f}".format(float(-msy))
    msrString = "{:7.3f}".format(float(msr))
    #print('m %s %s %s\n'%("000.000",msyString,msrString))
    ser.write(('m %s %s %s\n'%(msxString,msyString,msrString)).encode('utf-8'))
    await asyncio.sleep(0.005)
    
@sio.on('rot')
async def position(sid, yaw, pitch, roll):
    yawString = "{:7.3f}".format(float(yaw/6))
    pitchString = "{:7.3f}".format(float(pitch/6))
    rollString = "{:7.3f}".format(float(roll/6))
    ser.write(('r %s %s %s\n'%(rollString, pitchString, yawString)).encode('utf-8'))
    await asyncio.sleep(0.005)

@sio.on('pos')
async def position(sid, message):
    if message == 1:
        ser.write(('u\n').encode('utf-8'))
    if message == 0:
        ser.write(('d\n').encode('utf-8'))
    await asyncio.sleep(0.005)
        
@sio.on('gait')
async def position(sid, message):
    if message == 1:
        ser.write(('a\n').encode('utf-8'))
    elif message == 0:
        ser.write(('b\n').encode('utf-8'))
    await asyncio.sleep(0.005)
    
@sio.on('claw')
async def position(sid, message):
    if message == 1:
        ser.write(('c\n').encode('utf-8'))
    elif message == 0:
        ser.write(('o\n').encode('utf-8'))
    await asyncio.sleep(0.005)

@sio.on('power')
async def position(sid, message):
    if message == 1:
        print('power down')

app.router.add_get('/', index)

if __name__ == '__main__':
    web.run_app(app, host='0.0.0.0', port=3000)
    loop.run_forever()
    serialLoop.run_forever()
    
