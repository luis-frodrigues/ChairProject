import math
import numpy
import time
import csv
import json
import threading
# import context

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import Adafruit_TMP.TMP006 as TMP006
import RPi.GPIO as GPIO
import spidev
import Adafruit_DHT
import lsm303d


DEBUG = True


# Global lists for machine learning (for calculating means and deviations)
temp_all = []
tempctrl_all = []
hum_all = []
hum_deriv_all = []
humroom_all = []
accel_all = []
accel_deriv_all = []
bpm_all = []
bpm_deriv_all = []
press_all = []
press_deriv_all = []
press_2deriv_all = []


# Sensor reading flag
sensor_read = True

# Temperature
controltemp_Sensor = TMP006.TMP006()
bodytemp_Sensor = TMP006.TMP006(address=0x41)
ctrltemp_flag = 0
diftemp_flag = False

# Humidity
hum_sensor = Adafruit_DHT.DHT11
flag_hum = False

# Accelerometer
accel_Sensor = lsm303d.lsm303d() # 0x1D
flag_accel = False
flag_jerk = False

# Pressure
off_the_chair1 = 0
off_the_chair2 = 0

# Emotional State
emotion_state = "normal"


# mqtt credentials. Also make sure to change device_id at bottom of page
creds_bpress = {
    'clientId': 'pressure_b',
    'user':     'A1E-lRel1Ay8b5uI8pvruM6uHadB1v8Baq',
    'password': '',
    'topic':    '/v1.6/devices/pressure',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds_dpress = {
    'clientId': 'pressure_d',
    'user':     'A1E-4iJNU25nVLAeNUQrVU1R34ZXFKpdxi',
    'password': '',
    'topic':    '/v1.6/devices/pressure',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds_hum_temp = {
    'clientId': 'hum.temp',
    'user':     'A1E-l6ZnkD9p0CJ1OfNk2pw4gIi8sYK3tm',
    'password': '',
    'topic':    '/v1.6/devices/pressure',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds_accel = {
    'clientId': 'accel',
    'user':     'A1E-TjiSzIhMTa5EitFgugmG9ENttYCIOD',
    'password': '',
    'topic':    '/v1.6/devices/pressure',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds_bpm = {
    'clientId': 'bitpm',
    'user':     'A1E-zxNGSEdwIWd8JZRCcsmfd6kOzWdXVR',
    'password': '',
    'topic':    '/v1.6/devices/pressure',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}


# A delegate class providing callbacks for an MQTT client.
class MqttDelegate(object):
    
    def __init__(self, client, credentials):
        self.client = client
        self.credentials = credentials
    
    def on_connect(self, client, userdata, flags, rc):   
        if rc == 0:
            print 'Connected.'
        else :
            print 'Not connected'
        #self.client.subscribe(self.credentials['topic'].encode('utf-8'))
        self.client.subscribe(self.credentials['topic'] + 'cmd')
    
    def on_message(self, client, userdata, msg):
        if DEBUG:
            print 'Command received: %s' % msg.payload
    
    def on_publish(self, client, userdata, mid):
        if DEBUG:
            print 'Message published.'


# Connect to MQTT client
def ConnectToClient(client, delegate, credentials):
    client.on_connect = delegate.on_connect
    client.on_message = delegate.on_message
    client.on_publish = delegate.on_publish
    user, password = credentials['user'], credentials['password']
    client.username_pw_set(user, password)
    # client.tls_set(cafile)
    # client.tls_insecure_set(False)

    try:
        if DEBUG:
            print 'Connecting to mqtt server.'

        server, port = credentials['server'], credentials['port']
        client.connect(server, port, keepalive=120)
    except:
        print 'Connection failed, check your credentials!'
        return


# Sensor address setup
def SensorAddressSetup():
    controltemp_Sensor.begin()
    bodytemp_Sensor.begin(samplerate = TMP006.CFG_8SAMPLE)


def form(var):
    return '{0:0.3}'.format(var)


def GetDerivative(value, value_last, deriv_list, time_delta, it):
    deriv_list[it] = (value - value_last) / time_delta
    return deriv_list[it]


# To read from Analog-Digital Converter
def readadc(adcnum,spi):
    if adcnum > 7 or adcnum<0 :
        return -1
    r = spi.xfer2([1, 8+adcnum << 4, 0])
    data = ((r[1] & 3)<<8) +r[2]

    return data


# Initialize Values
def InitializeValues(client_hum_temp, hum_bodyPin, maxHeartRate, minHeartRate, timeSpan, ecgMV, maxValues, minValues, spi):
    # Temp
    client_hum_temp.publish(topic="/v1.6/devices/test" , payload=json.dumps({"body_temperature": {"value": 0}}), qos=1, retain=False)

    # Humidity
    bodyHumidity, bodyTemperature = Adafruit_DHT.read_retry(hum_sensor, hum_bodyPin)
    hum_inittime = time.time()
    
    # Accel
    accel = accel_Sensor.getRealAccel()
    accel_list = [0 for i in range(10)]
    accel_list[9] = math.sqrt(accel[0]*accel[0] + accel[1]*accel[1])
    accel_inittime = time.time()

    # ECG
    # The first 5.2 seconds are needed to create a threshold for the max points to calculate the heart rate
    start = time.time()
    while (time.time()-start) < 5.2:
        for i in range(4):
            ecgMV = bitalino()
            maxHeartRate, minHeartRate = HeartRate(ecgMV, False, maxValues=maxValues, minValues=minValues)
            time.sleep(timeSpan)
    
    # Make the mean of values to get the threshold of max and min values
    maxHeartRate, minHeartRate = HeartRate(ecgMV, True, maxValues=maxValues, minValues=minValues)

    # Pressure
    fsr_value_last = [0 for i in range(4)]
    for i in range(4):
        fsr_value_last[i] = readadc(i,spi)
    press_inittime = time.time()

    return bodyHumidity, hum_inittime, accel_list, accel_inittime, maxHeartRate, minHeartRate, fsr_value_last, press_inittime


# Get Sensor Temperature
def GetSensorTemp(client):
    
    global ctrltemp_flag
    global diftemp_flag
    
    body_temp = bodytemp_Sensor.readObjTempC()

    if ctrltemp_flag / 15 == 0:
        ctrl_temp = controltemp_Sensor.readObjTempC()
        ctrltemp_flag = 0
    ctrltemp_flag += 1
    
    dif_temp = body_temp - ctrl_temp
            
    if dif_temp < 3 and diftemp_flag:
        message = {"body_temperature": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
        body_temp = 0
        diftemp_flag = False

    elif dif_temp > 3:
        if not diftemp_flag:
            message = {"body_temperature": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            diftemp_flag = True

        message = {"body_temperature": {"value": form(body_temp)}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )

        # DEBUG
        if DEBUG:
            print "Temp sent"

    return body_temp, ctrl_temp


# Get Sensor Humidity
def getHumidity(client, hum_bodyPin, hum_RoomPin, bodyHumidity_last, init_time, file):
    
    global flag_hum

    bodyHumidity, bodyTemperature = Adafruit_DHT.read_retry(hum_sensor, hum_bodyPin)
    roomHumidity, roomTemperature = Adafruit_DHT.read_retry(hum_sensor, hum_RoomPin)
    print 'Humidities:', bodyHumidity, roomHumidity
    
    end_time = time.time()
    time_delta = end_time - init_time
    bodyHum_derivative = (bodyHumidity - bodyHumidity_last) / time_delta

    difhum = bodyHumidity-roomHumidity

    if difhum < 5 and flag_hum:
        message = {"body_Humidity": {"value": 0} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        bodyHumidity=0
        flag_hum=False
        
    elif difhum > 5:
        if not flag_hum:
            message = {"body_Humidity": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_hum = True
            
        message = {"body_Humidity": {"value": form(bodyHumidity)}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
	
	init_time = end_time

    return bodyHumidity, roomHumidity, bodyHum_derivative, init_time


# Get Acceleration
def GetAcceleration(client, accel_list, jerk_list, init_time, file, cloud_flag):

    global flag_accel
    global flag_jerk

    accel = accel_Sensor.getRealAccel()
    end_time = time.time()
    time_delta = end_time - init_time
    
    # Gives positon of chair in xy
    # x_avgaccel = (accel[0] + accel_last[0])*490.05 # 9.801*100/2
    # y_avgaccel = (accel[1] + accel_last[1])*490.05

    # # DEBUG
    # if DEBUG:
    #     print "az = ", accel[2]

    # if x_avgaccel > 0.5:
    #     x_delta = x_avgaccel * time_delta * time_delta / 2
    #     x_pos += x_delta + x_initvel * time_delta
    #     x_initvel += x_avgaccel * time_delta

    #     # DEBUG
    #     if DEBUG:
    #         print "X = ", x_pos, "     Vx = ", x_initvel, "     ax = ", x_avgaccel

    # if y_avgaccel > 0.5:
    #     y_delta = y_avgaccel * time_delta * time_delta / 2
    #     y_pos += y_delta + y_initvel * time_delta
    #     y_initvel += y_avgaccel * time_delta

    #     # DEBUG
    #     if DEBUG:
    #         print "Y = ", y_pos, "     Vy = ", y_initvel, "     ay = ", y_avgaccel

    # Substitutes the oldest value recorded with the most recent
    accel_list[cloud_flag] = math.sqrt(accel[0]*accel[0] + accel[1]*accel[1])
    
    ###############################errado
    jerk_list[cloud_flag] = GetDerivative(accel_list[cloud_flag], accel_list[cloud_flag-1], jerk_list, time_delta, cloud_flag)
    ###############################errado

    if cloud_flag == 9:
        accel_mean = numpy.mean(accel_list)
        jerk_mean = numpy.mean(jerk_list)

        if accel_mean < 0.1 and flag_accel:
            message = {"accel_mean": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
            accel_mean = 0
            flag_accel = False      
        elif accel_mean > 0.1:
            if not flag_accel:
                message = {"accel_mean": {"value": 0}}
                client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
                flag_accel = True

            message = {"accel_mean": {"value": form(accel_mean)} }
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )

        # if jerk_mean < 0.1 and flag_jerk:
        #     message = {"jerk_mean": {"value": 0}}
        #     client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        #     jerk_mean = 0
        #     flag_jerk = False      
        # elif jerk_mean > 0.1:
        #     if not flag_jerk:
        #         message = {"jerk_mean": {"value": 0}}
        #         client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
        #         flag_jerk = True

        # message = {"jerk_mean": {"value": form(jerk_mean)} }
        # client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )

    init_time = end_time

    # DEBUG
    if DEBUG:
        print "Accel/jerk sent"

    file.write("%f," % accel_list[cloud_flag])
    file.write("%f," % jerk_list[cloud_flag])

    return init_time


# Get Bitalino mV
def bitalino():
    spi = spidev.SpiDev()
    spi.open(0,0)
    bitalino = readadc(5,spi)
    ecgV = (bitalino*3.3/2**10-3.3/2)/1100 # value in volts
    ecgMV = ecgV*1000 # value in milivolts
    
    return ecgMV


# Gives the max and min values that will be used for the tresholds
def HeartRate(heartValue=0, TimeEnd=False, maxValues=[], minValues=[]):   
    minHeartRate=0
    maxHeartRate=0
    
    if not TimeEnd:
        maxValues.append(float(heartValue)**3)
    else:
        maxValues.sort(reverse=True)
        minValues= maxValues[-4:-1]        
        maxValues=maxValues[:4]
        # Calculates the mean values of the vector for the max and min values
        minHeartRate=numpy.mean(minValues)
        maxHeartRate=numpy.mean(maxValues)
        return maxHeartRate,minHeartRate    
    return maxHeartRate, minHeartRate


# Calculates the mean value of heart rate from the last 4 values recorded
def HeartRateAverage(ecgMV, minHeartRate, maxHeartRate, vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, i ,rate):
    if(ecgMV <= 0.8*minHeartRate):
        MinimumTime= iteration*timeSpan
                   
    if(ecgMV >= 0.85*maxHeartRate and (iteration*timeSpan)-actualTime > 0.5 and (iteration*timeSpan)-MinimumTime < 0.35):
        i+=1
        oldTime=actualTime
        actualTime= iteration*timeSpan
        
        if i!=0:
            vec.append(1/(float(actualTime)-float(oldTime))*60)
    
        if i>3:
            # Stores the actual and the previous mean value of the 4 maximums recorded
            rate[0]= rate[1]
            rate[1]=numpy.mean(vec[i-4:i-1])
 
    # Iteration is used to count the time and to calculate the heart rate
    iteration=+1
    # Time between the actual and the previous maximum
    time_delta=float(actualTime)-float(oldTime)
    return time_delta, rate, iteration 


# Get Heart bpm
def GetHeartRate(client, file, ecgMV, minHeartRate, maxHeartRate, vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, max_count, bpm_rate):

    # Read ECG values for 10 x timeSpan seconds
    for x in range(1,10):
        ecgMV = bitalino()
        time_delta, bpm_rate, iteration= HeartRateAverage(ecgMV, minHeartRate, maxHeartRate, vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, max_count, bpm_rate)
        # Pace to read the sensor values 
        time.sleep(timeSpan)
    # Calculates the derivative between the actual and the previous heart rate mean value
    if time_delta==0:
		bpm_rate_derivative =0
    else:
        bpm_rate_derivative = (bpm_rate[1]-bpm_rate[0])/time_delta
    
    message = {"pulse": {"value": bpm_rate} } 
    client.publish(topic="/v6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
    #message = {"bpm_rate_derivative": {"value": form(bpm_rate_derivative)} } 
    #client.publish(topic="/v6/devices/test" , payload=json.dumps(message), qos=1, retain=False )  

    file.write("%f," % bpm_rate[1])
    file.write("%f," % bpm_rate_derivative)
    return iteration


# Get Sensor Pressure 
def getPressure(client_bpress, client_dpress, back_position_list, bottom_position_list, fsr_value_last, init_time, file, cloud_flag):
    
    global off_the_chair1
    global off_the_chair2
    global off_the_chair3
    global sensor_read

    fsr_value = []
    spi = spidev.SpiDev()
    spi.open(0,0)

    end_time = time.time()
    time_delta = end_time - init_time
    fsr_deriv_list = [0 for i in range(10)]
    
    # Read pressure values
    for i in range(4):
        fsr_value.append(readadc(i,spi))
        
        if DEBUG:
            print fsr_value[i]

        fsr_deriv_list[i] = GetDerivative(fsr_value[i], fsr_value_last[i], fsr_deriv_list, time_delta, i)

        file.write("%f," % fsr_value[i])
        file.write("%f," % fsr_deriv_list[i])

    # Back position
    if fsr_value[0] < 10 and fsr_value[3] < 10:
        back_position_list[cloud_flag] = 0
        off_the_chair1 += 1
    else:
        back_position_list[cloud_flag] = 7.5*(fsr_value[3] - fsr_value[0])/900

    # Bottom position
    if fsr_value[1] < 10 and fsr_value[2] < 10:
        bottom_position_list[cloud_flag] = 0
        off_the_chair2 += 1
    else:
        # 15 cm ??
        bottom_position_list[cloud_flag] = 7.5*(fsr_value[1] - fsr_value[2])/900

    if off_the_chair1 > 0 and off_the_chair2 > 0:
        sensor_read = False

    if cloud_flag == 9:
        if off_the_chair1 > 0:
             back_message = {"back_position_off": {"value": 0}}
             off_the_chair1 = 0
        else:
            back_message = {"back_position_mean": {"value": form(numpy.mean(back_position_list))}}
        if off_the_chair2 > 0:
             bottom_message = {"bottom_position_off": {"value": 0}}
             off_the_chair2 = 0
        else:
            bottom_message = {"bottom_position_mean": {"value": form(numpy.mean(bottom_position_list))}}

        client_bpress.publish(topic="/v1.6/devices/test" , payload=json.dumps(back_message), qos=1, retain=False)
        client_dpress.publish(topic="/v1.6/devices/test" , payload=json.dumps(bottom_message), qos=1, retain=False)

    init_time = end_time
    fsr_value_last = fsr_value

    return init_time


def my_state():
    global emotion_state
    while emotion_state != 'q':
        print 'Inserir novo estado: '
        emotion_state = raw_input()


########## MAIN ##########

def main():
    
    # Sensor reading flag/timer
    global sensor_read
    sensor_timer = 0

    # Humidity
    hum_bodyPin = 17
    hum_RoomPin = 18

    # Acceleration
    jerk_list = [0 for i in range(10)]

    # ECG
    maxValues=[]
    minValues=[]
    ecgMV=0.0
    maxHeartRate=0
    minHeartRate=0
    vec=[]
    timeSpan=0.1
    actualTime=0
    oldTime=0
    MinimumTime=0
    iteration=0
    max_count=0
    bpm_rate=[0.0, 0.0]

    # Pressure
    fsr_value = [0 for i in range(4)]
    spi = spidev.SpiDev()
    spi.open(0,0)
    back_position_list = [0 for i in range(10)]
    bottom_position_list = [0 for i in range(10)]
    
    # Cloud Clients
    client_bpress = mqtt.Client(client_id=creds_bpress['clientId'])
    delegate_bpress = MqttDelegate(client_bpress, creds_bpress)
    ConnectToClient(client_bpress, delegate_bpress, creds_bpress)
    client_dpress = mqtt.Client(client_id=creds_dpress['clientId'])
    delegate_dpress = MqttDelegate(client_dpress, creds_dpress)
    ConnectToClient(client_dpress, delegate_dpress, creds_dpress)
    client_hum_temp = mqtt.Client(client_id=creds_hum_temp['clientId'])
    delegate_hum_temp = MqttDelegate(client_hum_temp, creds_hum_temp)
    ConnectToClient(client_hum_temp, delegate_hum_temp, creds_hum_temp)
    client_accel = mqtt.Client(client_id=creds_accel['clientId'])
    delegate_accel = MqttDelegate(client_accel, creds_accel)
    ConnectToClient(client_accel, delegate_accel, creds_accel)
    client_bpm = mqtt.Client(client_id=creds_bpm['clientId'])
    delegate_bpm = MqttDelegate(client_bpm, creds_bpm)
    ConnectToClient(client_bpm, delegate_bpm, creds_bpm)
	
    SensorAddressSetup()
	
    file = open("data_sensors.txt", "a")

    bodyHumidity, hum_inittime, \
    accel_list, accel_inittime, \
    maxHeartRate, minHeartRate, \
    fsr_value_last, press_inittime = \
    InitializeValues(client_hum_temp, hum_bodyPin, maxHeartRate, minHeartRate, timeSpan, ecgMV, maxValues, minValues, spi)
    emotion = threading.Thread(name='my_state', target=my_state)
    emotion.start()
    
    if DEBUG:
        print "CICLO"

    ### EL CICLO ###

    while True:
        client_bpress.loop()
        client_dpress.loop()
        client_hum_temp.loop()
        client_accel.loop()
        client_bpm.loop()

        print "temp"
        body_temp, ctrl_temp = GetSensorTemp(client_hum_temp)
        
        print "humidity"
        bodyHumidity, roomHumidity, bodyHum_derivative, hum_inittime = getHumidity(client_hum_temp, hum_bodyPin, hum_RoomPin, bodyHumidity, hum_inittime, file)

        for k in range(10):
            # DEBUG
            if DEBUG:
                print "In Iteration", k
            
            print 'body_temp'
            file.write("%f," % body_temp)
            print 'ctrl_temp'
            file.write("%f," % ctrl_temp) 
            print 'bodyHumidity'
            file.write("%f," % bodyHumidity)
            print 'bodyHum_derivative'
            file.write("%f," % bodyHum_derivative)
            print 'roomHumidity'
            file.write("%f," % roomHumidity)
			
            print 'acceleration'      
            accel_inittime = GetAcceleration(client_accel, accel_list, jerk_list, accel_inittime, file, k)
            print "bitalino"
            iteration=GetHeartRate(client_bpm, file, ecgMV, minHeartRate, maxHeartRate, vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, max_count, bpm_rate=bpm_rate)
            print "pressure"
            press_inittime = getPressure(client_bpress, client_dpress, back_position_list, bottom_position_list, fsr_value_last, press_inittime, file, k)
            
            file.write(emotion_state)
            file.write("\n")
        
            # DEBUG
            if DEBUG:
                print 'emotion_state', emotion_state
                print 'sensor_read: ', sensor_read
            
            # If person is not seated, do not aquire values. After 30 seconds, turn off program
            if emotion_state == 'q':
                # file.seek(0,0)
                file.close()
                emotion.join()
                return 1
            
            if not sensor_read:
                timer = time.time()
            
                while not sensor_read:
                    if time.time()-timer > 30 :
                        # file.seek(0,0)
                        file.close()
                        emotion.join()
                        return 1
                        
                    for i in range(4):
                        fsr_value[i] = readadc(i,spi)
                        if fsr_value[i] > 10:
                            sensor_read = True
                            break

                    # DEBUG
                    if DEBUG:
                        print emotion_state, time.time()-timer
                        

        # DEBUG
        if DEBUG:
            print "Out Iteration"


if __name__ == '__main__':
	main()