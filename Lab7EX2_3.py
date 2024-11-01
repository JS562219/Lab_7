from imu import MPU6050
from time import sleep,sleep_ms,ticks_us,ticks_diff
from machine import Pin, I2C
from  IMUHelpers import printtab,printtabround,mpu,RAD_TO_DEG_CONSTANT
import math
i2c = I2C(1,sda=Pin(2),scl = Pin(3), freq = int(400E3))
imu = MPU6050(i2c)
tau = 0.895

led = Pin("LED", Pin.OUT)


file = open("DataPoints.csv",'w')
def imu_setup():
    global i2c, imu,file
    cal_samples = 20
    
    
    imu.filter_range = 4
    imu.accel_range = 1
    imu.gyro_range = 0

    print("Calibrating, do not move!")

    for i in range(cal_samples):
        print(".",end="")

        mpu.acc_x_tare = (mpu.acc_x_tare + imu.accel.x)/2

        mpu.acc_y_tare = (mpu.acc_y_tare + imu.accel.y)/2
    
        mpu.acc_z_tare = (mpu.acc_z_tare - imu.accel.z)/2

        mpu.gyro_x_tare = (mpu.gyro_x_tare + imu.gyro.x)/2
        
        mpu.gyro_y_tare = (mpu.gyro_y_tare + imu.gyro.y)/2
    
        mpu.gyro_z_tare = (mpu.gyro_z_tare + imu.gyro.z)/2

        #delete these next 2 lines when finished adding code
        #raise ValueError("Calibration code not complete!!!!! (Should probably add more code)")
        
    
        sleep_ms(1)
    print()
    mpu.acc_z_tare -= 9.81
    print("Calibration complete...")
    print("Ax\t Ay\t Az\t Gx\t Gy\t Gz")
    printtabround(mpu.acc_x_tare)
    printtabround(mpu.acc_y_tare)
    printtabround(mpu.acc_z_tare)
    printtabround(mpu.gyro_x_tare)
    printtabround(mpu.gyro_y_tare)
    printtabround(mpu.gyro_z_tare)
    print()

    print("Raw read...")
    print("Ax\t Ay\t Az\t Gx\t Gy\t Gz")
    printtabround(imu.accel.x)
    printtabround(imu.accel.y)
    printtabround(-imu.accel.z)
    printtabround(imu.gyro.x)
    printtabround(imu.gyro.y)
    printtabround(imu.gyro.z)
    print()
    print("Calibrated read...")
    print("Ax\t Ay\t Az\t Gx\t Gy\t Gz")

    printtabround(mpu.acc_x_tare-imu.accel.x)
    printtabround(mpu.acc_y_tare-imu.accel.y)
    printtabround(mpu.acc_z_tare+imu.accel.z)
    printtabround(mpu.gyro_x_tare-imu.gyro.x)
    printtabround(mpu.gyro_y_tare-imu.gyro.y)
    printtabround(mpu.gyro_z_tare-imu.gyro.z)
    print()
    
    sleep_ms(4000)


def imu_loop():
    global i2c, imu, file
    OUTPUT_RAW6DOF   =               not True
    OUTPUT_TARE6DOF        =         not True
    OUTPUT_GYRO_INTEGRAL      =      not True
    OUTPUT_GYRO_CALIBRATED_INTEGRAL= not True
    OUTPUT_ACC_VECTOR            =    not True
    OUTPUT_ACC_VECTOR_CALIBRATED =    not True
    OUTPUT_COMPLEMENATARY_FILTER =    True
    last_time = ticks_us()
    while 1:
        
        micros_now = ticks_us()
        dt = (ticks_diff(micros_now, last_time)) / 1e6
        last_time = micros_now
        
        if OUTPUT_RAW6DOF:
            printtabround(imu.accel.x)
            printtabround(imu.accel.y)
            printtabround(-imu.accel.z)
            printtabround(imu.gyro.x)
            printtabround(imu.gyro.y)
            printtabround(imu.gyro.z)
    
            file.write("\n")
            file.write(str(round(imu.accel.x,2))+"\t") 
            file.write(str(round(imu.accel.y,2))+"\t")
            file.write(str(round(imu.accel.z,2))+"\t")
            file.write(str(round(imu.gyro.x,2))+"\t")
            file.write(str(round(imu.gyro.y,2))+"\t")
            file.write(str(round(imu.gyro.z,2))+"\t")
            print()
            file.write("\n")
            sleep_ms(1)

        if OUTPUT_TARE6DOF:
            printtabround(mpu.acc_x_tare-imu.accel.x)
            printtabround(mpu.acc_y_tare-imu.accel.y)
            printtabround(mpu.acc_z_tare+imu.accel.z)
            printtabround(mpu.gyro_x_tare-imu.gyro.x)
            printtabround(mpu.gyro_y_tare-imu.gyro.y)
            printtabround(mpu.gyro_z_tare-imu.gyro.z)

            file.write("\n")
            file.write(str(round((mpu.acc_x_tare-imu.accel.x),2))+"\t")
            file.write(str(round((mpu.acc_y_tare-imu.accel.y),2))+"\t")
            file.write(str(round((mpu.acc_z_tare+imu.accel.z),2))+"\t")
            file.write(str(round((mpu.gyro_x_tare-imu.gyro.x),2))+"\t")
            file.write(str(round((mpu.gyro_y_tare-imu.gyro.y),2))+"\t")
            file.write(str(round((mpu.gyro_z_tare-imu.gyro.z),2))+"\t")
            file.write("\n")
            print()
            sleep_ms(1)
            
        #Gyroscope integration only
        if(OUTPUT_GYRO_INTEGRAL):
            #save this because of integration
            raw_g_pitch_angle_rad = 0
            #do the pitch calc
            raw_g_pitch_angle_rad += imu.gyro.y * dt
            printtabround(raw_g_pitch_angle_rad * RAD_TO_DEG_CONSTANT)
            #ROLL????
            raw_g_roll_angle_rad = 0
            #do the pitch calc
            raw_g_roll_angle_rad += imu.gyro.x * dt
            printtabround(raw_g_roll_angle_rad * RAD_TO_DEG_CONSTANT)
            file.write("\n")
            file.write(str(round((raw_g_pitch_angle_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write(str(round((raw_g_roll_angle_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            print()
            sleep_ms(1)

            
  
        #Gyroscope integration with calibration
        if(OUTPUT_GYRO_CALIBRATED_INTEGRAL):
        
            #save this because of integration
            pitch_angle_rad = 0
            roll_angle_rad = 0
            #do the pitch calc
            pitch_angle_rad += (mpu.gyro_y_tare - imu.gyro.y) * dt
            printtabround(pitch_angle_rad * RAD_TO_DEG_CONSTANT)
            #ROLL????
            roll_angle_rad += (mpu.gyro_x_tare - imu.gyro.x) * dt
            printtabround(roll_angle_rad * RAD_TO_DEG_CONSTANT)

            file.write("\n")
            file.write(str(round((pitch_angle_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write(str(round((roll_angle_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write("\n")
            print()
            sleep_ms(1)

        #Find the acclerometer vector
        if(OUTPUT_ACC_VECTOR):
        
            raw_acc_x = imu.accel.x
            raw_acc_y = imu.accel.y
            #need to invert Z if the sensor is facing down
            raw_acc_z = -imu.accel.z

            accelPitch_rad  = -math.atan2(raw_acc_y, raw_acc_z)
            printtabround(accelPitch_rad * RAD_TO_DEG_CONSTANT)
            accelRoll_rad = -math.atan2(raw_acc_x, raw_acc_z)
            printtabround(accelRoll_rad * RAD_TO_DEG_CONSTANT)

            file.write("\n")
            file.write(str(round((accelPitch_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write(str(round((accelRoll_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write("\n")
            print()
            sleep_ms(1)
    
        #Find the acclerometer vector
        if(OUTPUT_ACC_VECTOR_CALIBRATED):
        
            
            Craw_acc_y = mpu.acc_y_tare - imu.accel.y
            Craw_acc_x = mpu.acc_x_tare - imu.accel.x
            #tare is added to the z reading to handle sensor upside down
            Craw_acc_z = mpu.acc_z_tare + imu.accel.z

            #z is *-1 to change output to +/- 0 instead of +/- 180
            accelPitch_rad  = math.atan2(Craw_acc_y, -Craw_acc_z)
            printtabround(accelPitch_rad * RAD_TO_DEG_CONSTANT)
            #ROLL????
            accelRoll_rad  = math.atan2(Craw_acc_x, -Craw_acc_z)
            printtabround(accelRoll_rad * RAD_TO_DEG_CONSTANT)

            file.write("\n")
            file.write(str(round((accelPitch_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write(str(round((accelRoll_rad * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write("\n")
            print()
            sleep_ms(1)
        
        #Complemetary filter goes here
        if(OUTPUT_COMPLEMENATARY_FILTER):

            Craw_acc_y = mpu.acc_y_tare - imu.accel.y
            Craw_acc_x = mpu.acc_x_tare - imu.accel.x
            #tare is added to the z reading to handle sensor upside down
            Craw_acc_z = mpu.acc_x_tare + imu.accel.z

            #Find the accleration vector angles (Z/Y and Z/X)
            pitch_angle_rad = (mpu.gyro_y_tare - imu.gyro.y) * dt
            roll_angle_rad = (mpu.gyro_x_tare - imu.gyro.x) * dt
            accelPitch_rad  = -math.atan2(Craw_acc_y, Craw_acc_z)
            accelRoll_rad = -math.atan2(Craw_acc_x, Craw_acc_z)
            Pitch = tau*(accelPitch_rad+pitch_angle_rad)+(1-tau)*(accelPitch_rad)
            Roll = tau*(accelRoll_rad+roll_angle_rad)+(1-tau)*(accelRoll_rad)

            printtabround(Pitch * RAD_TO_DEG_CONSTANT)
            printtabround(Roll * RAD_TO_DEG_CONSTANT)

            file.write("\n")
            file.write(str(round((Pitch * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write(str(round((Roll * RAD_TO_DEG_CONSTANT),2))+"\t")
            file.write("\n")
            print()
            sleep_ms(1)

            P = Pitch * RAD_TO_DEG_CONSTANT
            R = Roll * RAD_TO_DEG_CONSTANT

            led.off()
            while P > 45 or P < -45:
                led.on()
                if P > 45 and -4 <= R <= 4:
                    led.off()
            else:
                led.on()
            

def program():
    imu_setup()
    imu_loop()

program()