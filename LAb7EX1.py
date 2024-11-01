from imu import MPU6050
from time import sleep,sleep_ms
from machine import Pin, I2C
from IMUHelpers import *
i2c = I2C(0,sda=Pin(0),scl = Pin(1), freq = int(400E3))
imu = MPU6050(i2c)

RAD_TO_DEG_CONSTANT = 57.29746936176985516473022441508
file = open("DataPoints.csv",'w')
def imu_setup():
    global i2c, imu,file
    
    imu.filter_range = 4
    imu.accel_range = 1
    imu.gyro_range = 0
    
    printtab("Ax")
    printtab("Ay")
    printtab("Az")
    printtab("Gx")
    printtab("Gy")
    printtab("Gz")
    file.write("Ax"+"\t")
    file.write("Ay"+"\t")
    file.write("Az"+"\t")
    file.write("Gx"+"\t")
    file.write("Gy"+"\t")
    file.write("Gz"+"\t")
    print()
    file.write("\n")

def imu_loop():
    global imu,file
    #code for writing files
    
    while 1:
        
        printtabround(imu.accel.x)
        printtabround(imu.accel.y)
        printtabround(-imu.accel.z)
        printtabround(imu.gyro.x)
        printtabround(imu.gyro.y)
        printtabround(imu.gyro.z)
        
        file.write(str(round(imu.accel.x,2))+"\t")
        file.write(str(round(imu.accel.y,2))+"\t")
        file.write(str(round(imu.accel.z,2))+"\t")
        file.write(str(round(imu.gyro.x,2))+"\t")
        file.write(str(round(imu.gyro.y,2))+"\t")
        file.write(str(round(imu.gyro.z,2))+"\t")
        print()
        file.write("\n")
        sleep_ms(1)



def program():
    imu_setup()
    imu_loop()

program()