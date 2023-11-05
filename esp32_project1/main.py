import umqtt_robust2 as mqtt
from machine import Pin, UART, I2C, ADC
from gps_bare_minimum import GPS_Minimum
from time import sleep
from imu import MPU6050
from neopixel import NeoPixel

def main():
    mqttVar = 0
    loopCounter = 0
    
    bat_adc = ADC(34)                      
    bat_adc.atten(ADC.ATTN_11DB)          
    bat_adc.width(ADC.WIDTH_10BIT)
    bat_scaling = 4.2 / 830 
    
    def read_battery_voltage_avg64():                  
        adc_val = 0
        for i in range(64):
            adc_val += bat_adc.read()      
        voltage = bat_scaling * (adc_val >> 6)                   
        adc_value64 = adc_val >> 6
        battery_percentage = ((voltage - 3)/1.1)*100 
        return battery_percentage                             

    i2c = I2C(0)
    imu = MPU6050(i2c)
    tackling = 1
    
    gps_port = 2
    gps_speed = 9600
    uart = UART(gps_port, gps_speed)            
    gps = GPS_Minimum(uart)
    
    np = NeoPixel(Pin(26, Pin.OUT), 12)
    max1 = False
    
    def movePixel(x, r, g ,b):
        for i in range(1, x):
            np[i] = (r,g,b)
            np.write()
            
    for i in range(12):
        np[i] = (0,0,0)
        np.write()
        
    sleep(1)
    
    while True:
        try:
            loopCounter = loopCounter + 1
            print('')
            print('---------')
            print('LOOPCOUNTER ', loopCounter)
            if loopCounter == 59:
                mqttVar = 1
                loopCounter = 0
            if loopCounter == 29:
                mqttVar = 0
                
            sleep(1)
            print('')
            print('- BATTERI')
            
            batteryVal = read_battery_voltage_avg64()
            
            if batteryVal < 0:
                batteryVal = 0
            elif batteryVal > 100:
                batteryVal = 100
                
            if batteryVal < 10:
                np[0] = (10,0,0)
                np.write()
            elif int(batteryVal) >= 10 and int(batteryVal) <= 45:
                np[0] = (20,20,0)
                np.write()
            else:
                np[0] = (0,10,0)
                np.write()
                
            print('battery percentage =', batteryVal)
            batteryMsg1 = '%'
            batteryMsg = str(batteryVal) + str(batteryMsg1)
            if mqttVar == 0:    
                mqtt.web_print(batteryMsg, 'MateoFP/feeds/batteryFeed')
                print('')
                mqttVar = 3

            acceleration = imu.accel
            print("")
            print("- IMU")
            if (acceleration.y > 0.6) & (acceleration.y > 0):
                print("Y axis points upwards")
            else:
                if tackling <= 12:
                    sleep(1)
                    if (acceleration.y > 0.6) & (acceleration.y > 0):
                            print("Y axis points upwards, tackle not counted")
                    else:
                        tackling += 1
                        while (acceleration.y < 0.8):
                            print("spilleren er blevet tacklet og er stadig nede.")
                            sleep(0.100)
                else:
                    print('max tacklinger. [11]')
            print('')
            print("- TACKLINGER")
            print(tackling - 1)
            if max1 == True:
                if tackling != 1:
                    if tackling >= 12:
                        movePixel(12, 0, 5, 5)
                    else:
                        movePixel(tackling, 0, 5, 5)
                print('')
            elif tackling != 1:
                if tackling >= 12:
                    max1 = True
                    movePixel(tackling, 0, 0, 5)
                    tackling = 1
                else:
                    movePixel(tackling, 0, 0, 5)
            print('')
            
            if mqttVar == 1:     
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
                        mqttVar = 3
                    else:
                        print("error: GPS data is invalid.")
                        mqtt.web_print("error: GPS data is invalid.")
                        mqttVar = 3
                else:
                        print('error: GPS data is not available.')
                        mqtt.web_print('error: GPS data is unavailable.')
                        mqttVar = 3
                
            #if read_battery_voltage_avg64() <= 5:
                #return
    
            mqtt.sync_with_adafruitIO()
        
        except KeyboardInterrupt: 
            print('Ctrl-C pressed...exiting')
            mqtt.c.disconnect()
            mqtt.sys.exit()
            
if __name__ == '__main__':
    main()
