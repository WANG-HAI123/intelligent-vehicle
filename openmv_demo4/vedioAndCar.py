THRESHOLD = (1, 11, -16, 7, -16, 7) # Grayscale threshold for dark things...
import sensor, image, time, pyb, gc
from pyb import LED
import car
from mypid import PID

rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.001, i=0)
FRAME_HEADER = b'IMG_START'
FRAME_FOOTER = b'IMG_END'

LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

uart = pyb.UART(3, 115200, timeout_char=1000)

while(True):
    clock.tick()
    img = sensor.snapshot()
    trans_img =  img.copy()
    line = img.binary([THRESHOLD]).get_regression([(100,100)], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        if line.magnitude()>8:
            #if -40<b_err<40 and -30<t_err<30:
            rho_output = rho_pid.get_pid(rho_err,1)
            theta_output = theta_pid.get_pid(theta_err,1)
            output = rho_output+theta_output
            print(output,rho_err,rho_err)

            car.run(80+output, 80+output)
        else:
            car.run(0,0)
    else:
        car.run(30,-30)
        pass
    compressed = trans_img.compress(quality=20)
    checksum = sum(compressed) % 256

    uart.write(FRAME_HEADER)
    uart.write(len(compressed).to_bytes(4, 'big'))
    uart.write(bytes([checksum]))
    uart.write(compressed)
    uart.write(FRAME_FOOTER)
    #print(clock.fps())
