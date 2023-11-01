import umqtt_robust2 as mqtt
from machine import Pin, UART, I2C, SoftI2C, ADC
from gps_bare_minimum import GPS_Minimum
from time import sleep
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
    bat_scaling = 4.2 / 4095
    
    def read_battery_voltage():
        adc_val = bat_adc.read()
        voltage = bat_scaling * adc_val
        return voltage
    
    def read_battery_voltage_avg64():      # Option: average over N times to remove fluctuations
        adc_val = 0
        for i in range(64):
            adc_val += bat_adc.read()      
        voltage = bat_scaling * (adc_val >> 6) # >> fast divide by 64
        return voltage
    
    #OXIMETER
    i2c = SoftI2C(sda=Pin(22), scl=Pin(21), freq=400000)
    sensor = MAX30102(i2c=i2c)
    
    if sensor.i2c_address not in i2c.scan():
        print("Sensor not found.")
        return
    elif not (sensor.check_part_id()):
        # Check that the targeted sensor is compatible
        print("I2C device ID not corresponding to MAX30102 or MAX30105.")
        return
    else:
        print("Sensor connected and recognized.")
        
    sensor.setup_sensor()
    compute_frequency = True
    t_start = ticks_us()  # Starting time of the acquisition
    samples_n = 0  # Number of samples that have been collected
    
    #IMU
    i2c2 = I2C(0)
    imu = MPU6050(i2c2)
    tackling = 0
    
    #GPS
    gps_port = 2
    gps_speed = 9600
    uart = UART(gps_port, gps_speed)            
    gps = GPS_Minimum(uart)
    
    #NEOPIXEL
    np = NeoPixel(Pin(26, Pin.OUT), 12)
    
    def movePixel(x):
        for i in range(x):
            np[i] = (0,0,20)
            np.write()
            
    for i in range(12):
        np[i] = (0,0,0)
        np.write()
    
    while True:
        try:
            #IMU + NEOPIXEL
            acceleration = imu.accel
            print("")
            print("- IMU")

            if abs(acceleration.y) > 0.8:
                if (acceleration.y > 0):
                    print("Y axis points upwards")
                else:               
                    print("Y axis points downwards")
                    if tackling < 12:
                        tackling += 1
                    while(abs(acceleration.y > 0.8) & abs(acceleration.y > 0) != True):
                        print("spilleren er blevet tacklet og er stadig nede.")
                        sleep(0.1)
            else:
                if tackling < 12:
                        tackling += 1
                while(abs(acceleration.y > 0.8) & abs(acceleration.y > 0) != True):
                    print("spilleren er blevet tacklet og er stadig nede.")
                    sleep(0.1) 
            print('')
            sleep(0.5)
            print("- TACKLINGER")
            print("%d" % tackling)
            print('')
            if tackling != 0:
                if tackling >= 12:
                    movePixel(12)
                else:
                    movePixel(tackling)
            
            #BATTERI
            print('- BATTERI')
            print('bat_volt    = ', read_battery_voltage())
            print('bat_volt_64 = ', read_battery_voltage_avg64())
            mqtt.web_print(read_battery_voltage_avg64(), 'MateoFP/feeds/batteryFeed')
            print('')
            sleep(4)
            
            #OXIMETER
            print("- OXIMETER")
            sensor.check()
            if sensor.available():
                red_reading = sensor.pop_red_from_storage()
                ir_reading = sensor.pop_ir_from_storage()
                print(red_reading, ",", ir_reading)
            if compute_frequency:
                if ticks_diff(ticks_us(), t_start) >= 999999:
                    f_HZ = samples_n
                    samples_n = 0
                    print("acquisition frequency = ", f_HZ)
                    t_start = ticks_us()
                else:
                    samples_n = samples_n + 1

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
                    mqtt.web_print(gps_location, 'MateoFP/feeds/mapFeed/csv')
                    sleep(4)
                else:
                    print("error: GPS data is invalid.")
                    mqtt.web_print("error: GPS data is invalid.")
                    sleep(4)
            else:
                print("error: no GPS data received.")
                mqtt.web_print("error: no GPS data received.")
                sleep(4)
                
            #IMU + NEOPIXEL 2
            acceleration = imu.accel
            print("")
            print("- IMU")

            if abs(acceleration.y) > 0.8:
                if (acceleration.y > 0):
                    print("Y axis points upwards")
                else:               
                    print("Y axis points downwards")
                    if tackling < 12:
                        tackling += 1
                    while(abs(acceleration.y > 0.8) & abs(acceleration.y > 0) != True):
                        print("spilleren er blevet tacklet og er stadig nede.")
                        sleep(0.1)
            else:
                if tackling < 12:
                        tackling += 1
                while(abs(acceleration.y > 0.8) & abs(acceleration.y > 0) != True):
                    print("spilleren er blevet tacklet og er stadig nede.")
                    sleep(0.1) 
            #NEOPIXEL
            print('')
            sleep(0.5)
            print("- TACKLINGER")
            print("%d" % tackling)
            print('')
            if tackling != 0:
                if tackling >= 12:
                    movePixel(12)
                else:
                    movePixel(tackling)
        
            if len(mqtt.besked) != 0: 
                mqtt.besked = ""
            
            mqtt.sync_with_adafruitIO()
        
        except KeyboardInterrupt: 
            print('Ctrl-C pressed...exiting')
            mqtt.c.disconnect()
            mqtt.sys.exit()
            
if __name__ == '__main__':
    main()
            
