import umqtt_robust2 as mqtt
from machine import Pin, UART, I2C, SoftI2C, ADC
from gps_bare_minimum import GPS_Minimum
from time import sleep, time
from imu import MPU6050
from neopixel import *
from utime import ticks_diff, ticks_us
from max30102 import MAX30102

#some code from https://github.com/KevinLindemark/E2023_GPS_micropython_ESP32/blob/main/GPS%20EKSEMPEL%202/adafruit_gps_main.py
#some code from "twoway_remote_data.py" af Bo Hansen

def main():
    #BATTERI
    bat_adc = ADC(34)                      
    bat_adc.atten(ADC.ATTN_11DB)           # Full range: 3,3 V
    bat_adc.width(ADC.WIDTH_10BIT)
    bat_scaling = 4.2 / 830 #
    
    def read_battery_voltage_avg64():      # Option: average over N times to remove fluctuations
        adc_val = 0
        for i in range(64):
            adc_val += bat_adc.read()      
        voltage = bat_scaling * (adc_val >> 6) # >> fast divide by 64
        adc_value64 = adc_val >> 6
        battery_percentage = ((voltage - 3)/1.1)*100 
        return battery_percentage #retunere batteri procent

    #OXIMETER
    i2c = SoftI2C(sda=Pin(22), scl=Pin(21), freq=400000)
    sensor = MAX30102(i2c=i2c)
    if sensor.i2c_address not in i2c.scan():
        print("Sensor not found.")
    elif not (sensor.check_part_id()):
        # Check that the targeted sensor is compatible
        print("I2C device ID not corresponding to MAX30102 or MAX30105.")
    else:
        print("Sensor connected and recognized.")
        
    sensor.setup_sensor()
    
    #IMU
    i2c2 = I2C(0)
    imu = MPU6050(i2c2)
    tackling = 1
    
    #GPS
    gps_port = 2
    gps_speed = 9600
    uart = UART(gps_port, gps_speed)            
    gps = GPS_Minimum(uart)
    
    #NEOPIXEL
    np = NeoPixel(Pin(26, Pin.OUT), 12)
    
    def movePixel(x, r, g ,b):
        for i in range(1, x):
            np[i] = (r,g,b)
            np.write()
            
    for i in range(12):
        np[i] = (0,0,0)
        np.write()
    sleep(1)
    startTime = time()

    while True:
        try:
            sleep(0.1)
            endTime = time() - startTime
            minutesFLOAT = float(endTime / 60)
            minutesINT = int(endTime / 60)
            remainder = minutesFLOAT - minutesINT
            seconds1 = remainder * 60
            print('-- Tidspunkt: %d sec' % (seconds1))
            
            #IMU + NEOPIXEL
            acceleration = imu.accel
            print("")
            print("- IMU")
            if (acceleration.y > 0.6) & (acceleration.y > 0):
                print("Y axis points upwards")
            else:
                if tackling <= 13:
                    sleep(1.5)
                    if (acceleration.y > 0.6) & (acceleration.y > 0):
                            print("Y axis points upwards, tackle not counted")
                    else:
                        tackling += 1
                        while (acceleration.y < 0.8):
                            print("spilleren er blevet tacklet og er stadig nede.")
                            sleep(0.1)
                else:
                    print('max tacklinger. [11]')
            print('')
            print("- TACKLINGER")
            print(tackling - 1)
            print('')
            if tackling != 1:
                if tackling >= 13:
                    movePixel(12, 0, 0, 10)
                else:
                    movePixel(tackling, 0, 0, 10)
            else:
                print("tacklinger = 0")
            
            #BATTERI
            print('- BATTERI')
            batteryVal = read_battery_voltage_avg64()
            if batteryVal < 0:
                batteryVal = 0
            elif batteryVal > 100:
                batteryVal = 100
                
            if batteryVal < 10:
                np[0] = (10,0,0)
                np.write()
            elif int(batteryVal) > 10 and int(batteryVal) < 45:
                np[0] = (20,20,0)
                np.write()
            else:
                np[0] = (0,10,0)
                np.write()
            print(' ')
            
            print('battery percentage =', batteryVal)
            batteryMsg1 = '%'
            batteryMsg = str(read_battery_voltage_avg64()) + str(batteryMsg1)
            if seconds1 = 30:    
                mqtt.web_print(batteryMsg, 'MateoFP/feeds/batteryFeed')
                print('')
            
            #OXIMETER
            print("- OXIMETER")
            sensor.check()
            if sensor.available():
                red_reading = sensor.pop_red_from_storage()
                ir_reading = sensor.pop_ir_from_storage()
                print(red_reading, ",", ir_reading)
            print(' ')
            #GPS 
            if gps.receive_nmea_data():
                if gps.get_validity() != 'V':
                    latitude = str(gps.get_latitude())
                    longitude = str(gps.get_longitude())
                    speed = str(gps.get_speed())
                    gps_location = speed + ',' + latitude + ',' + longitude + ',' + '0.0'
                    print("- GPS")
                    print('speed     = ' + speed + '\n' + 'latitude  = ' + latitude + '\n' + 'longitude = ' + longitude)
                    print()
                    if seconds1 < 1:
                        mqtt.web_print(gps_location, 'MateoFP/feeds/mapFeed/csv')
                else:
                    if seconds1 < 1:
                        print("error: GPS data is invalid.")
                        mqtt.web_print("error: GPS data is invalid.")
            else:
                if seconds1 < 1:
                    print("error: no GPS data received.")
                    mqtt.web_print("error: no GPS data received.")
                
            if len(mqtt.besked) != 0: 
                mqtt.besked = ""
                
            #IMU + NEOPIXEL
            acceleration = imu.accel
            print("")
            print("- IMU")
            if (acceleration.y > 0.6) & (acceleration.y > 0):
                print("Y axis points upwards")
            else:
                if tackling <= 13:
                    sleep(1.5)
                    if (acceleration.y > 0.6) & (acceleration.y > 0):
                            print("Y axis points upwards, tackle not counted")
                    else:
                        tackling += 1
                        while (acceleration.y < 0.8):
                            print("spilleren er blevet tacklet og er stadig nede.")
                            sleep(0.1)
                else:
                    print('max tacklinger. [11]')
                    
            print('')
            print("- TACKLINGER")
            print(tackling - 1)
            print('')
            if tackling != 1:
                if tackling >= 13:
                    movePixel(12, 0, 0, 10)
                else:
                    movePixel(tackling, 0, 0, 10)
            else:
                print("tacklinger = 0")
            
            mqtt.sync_with_adafruitIO()
        
        except KeyboardInterrupt: 
            print('Ctrl-C pressed...exiting')
            mqtt.c.disconnect()
            mqtt.sys.exit()
            
if __name__ == '__main__':
    main()
            
