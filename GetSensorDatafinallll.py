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


# DO NOT try to set values under 200 ms or the server will kick you out
publishing_period = 1000

# Temperature
controltemp_Sensor = TMP006.TMP006()
bodytemp_Sensor = TMP006.TMP006(address=0x44)
ctrltemp_flag = 0
diftemp_flag = False

# Humidity
hum_sensor = Adafruit_DHT.DHT11
flag_hum = False

# Accelerometer
accel_Sensor = lsm303d.lsm303d() # 0x1D

# Emotional State
emotion_state = "normal"


# mqtt credentials. Also make sure to change device_id at bottom of page
creds1 = {
    'clientId': '<pressure_b>',
    'user':     '<A1E-lRel1Ay8b5uI8pvruM6uHadB1v8Baq>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds2 = {
    'clientId': '<pressure_d>',
    'user':     '<A1E-4iJNU25nVLAeNUQrVU1R34ZXFKpdxi>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds3 = {
    'clientId': '<pressure_ds>',
    'user':     '<A1E-M21Kzt47fNVoZQSgQeUp0KWWaJ7qA0>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds4 = {
    'clientId': '<hum.temp>',
    'user':     '<A1E-l6ZnkD9p0CJ1OfNk2pw4gIi8sYK3tm>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds5 = {
    'clientId': '<accel>',
    'user':     '<A1E-TjiSzIhMTa5EitFgugmG9ENttYCIOD>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds6 = {
    'clientId': '<bitpm>',
    'user':     '<A1E-zxNGSEdwIWd8JZRCcsmfd6kOzWdXVR>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

# A delegate class providing callbacks for an MQTT client.
class MqttDelegate(object):
    
    def __init__(self, client, credentials):
        self.client = client
        self.credentials = credentials
    
    def on_connect(self, client, userdata, flags, rc):   
        if rc == 0 and DEBUG :
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
    client.username_pw_set(credentials['user'], credentials['user'])
    # client.tls_set(cafile)
    # client.tls_insecure_set(False)

    try:
        if DEBUG:
            print 'Connecting to mqtt server.'

        server, port = credentials['server'], credentials['port']
        client.connect("mqtt.things.ubidots.com", port=port, keepalive=120)
    except:
        print 'Connection failed, check your credentials!'
        return


# Sensor address setup
def SensorAddressSetup():
    controltemp_Sensor.begin()
    bodytemp_Sensor.begin(samplerate = TMP006.CFG_8SAMPLE)


def form(var):
    return '{0:0.3}'.format(var)


# To read from Analog-Digital Converter
def readadc(adcnum,spi):
    if adcnum > 7 or adcnum<0 :
        return -1
    r = spi.xfer2([1, 8+adcnum << 4, 0])
    data = ((r[1] & 3)<<8) +r[2]

    return data


# Initialize Values
def InitializeValues(client):
    # Temp
    client.publish(topic="/v1.6/devices/test" , payload=json.dumps({"body_temperature": {"value":0}}), qos=1, retain=False)

    # Accel
    accel_last = accel_Sensor.getRealAccel()
    init_time = time.time()

    # ECG
    # The first 5.2 seconds are needed to create a threshold for the max points to calculate then the heart rate
    start = time.time()
    while (time.time()-start) < 5.2:
        for i in range(1,5):
            ecgMV = bitalino(client)
            maxHeartRate, minHeartRate = HeartRate(ecgMV, False, maxValues=maxValues, minValues=minValues)
            time.sleep(timeSpan)
    
    # Make the mean of values to get the threshold of max and min values
    maxHeartRate, minHeartRate = HeartRate(ecgMV, True, maxValues=maxValues, minValues=minValues)

    return accel_last, init_time, maxHeartRate, minHeartRate


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
def getHumidity(client, hum_bodyPin, hum_RoomPin, file):
    
    global flag_hum

    bodyHumidity, bodyTemperature = Adafruit_DHT.read_retry(hum_sensor, hum_bodyPin)
    roomHumidity, bodyTemperature2 = Adafruit_DHT.read_retry(hum_sensor, hum_RoomPin)
    
    deltahum = bodyHumidity-roomHumidity

    if deltahum < 5 and flag_hum:
        message = {"dif_Humidity": {"value": 0} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        bodyHumidity=0
        flag_hum=False
        
    elif deltahum > 5:
        if not flag_hum:
            message = {"dif_Humidity": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_hum = True            
            
        message = {"dif_Humidity": {"value": form(deltahum)}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )               
    
    file.write("%f," % bodyHumidity)
    file.write("%f," % roomHumidity)


# Get Accelerometer Position
def GetAccelerometerPosition(client, accel_last, init_time, file):

    accel = accel_Sensor.getRealAccel()
    end_time = time.time()
    time_delta = end_time - init_time
    
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

    accel_squared = accel[0]*accel[0] + accel[1]*accel[1]
    accel_last_squared = accel_last[0]*accel_last[0] + accel_last[1]*accel_last[1]
    jerk = (accel_squared - accel_last_squared) / time_delta
    
    if (accel[0] < 0.1 or accel[0] > 0.1) and flag_acc:
        message = {"x_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        accel[0]=0
        flag_acc=False        
    elif accel[0] > 0.1 or accel[0] < 0.1:
        if not flag_acc:
            message = {"x_accel": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_acc = True

        message = {"x_accel": {"value": form(accel[0])} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )


    if (accel[0] < 0.1 or accel[0] > 0.1) and flag_acc:
        message = {"x_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        accel[0]=0
        flag_acc=False        
    elif accel[0]>0.1 or accel[0]<0.1:
        if not flag_acc:
            message = {"x_accel": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_acc = True

        message = {"x_accel": {"value": form(accel[0])} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
        
    if (accel[1]<0.1 or accel[1]>0.1) and flag_acc:
        message = {"y_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        accel[1]=0
        flag_acc=False        
    elif accel[1]>0.1 or accel[1]<0.1:
        if not flag_acc:
            message = {"y_accel": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_acc = True

        message = {"x_accel": {"value": form(accel[0])} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )

    if (accel[0]<0.1 or accel[0]>0.1) and flag_acc:
        message = {"x_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        accel[0]=0
        flag_acc=False        
    elif accel[0]>0.1 or accel[0]<0.1:
        if not flag_acc:
            message = {"x_accel": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_acc = True

        message = {"x_accel": {"value": form(accel[0])} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
        
    if (accel[1]<0.1 or accel[1]>0.1) and flag_acc:
        message = {"y_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        accel[1]=0
        flag_acc=False        
    elif accel[1]>0.1 or accel[1]<0.1:
        if not flag_acc:
            message = {"y_accel": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_acc = True

        message = {"y_accel": {"value": form(accel[0])} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )

    accel_last = accel
    init_time = end_time

    # DEBUG
    if DEBUG:
        print "Accel/jerk sent"

    file.write("%f," % accel_squared)
    file.write("%f," % jerk)

    time.sleep(0.2)


# Get Bitalino mV
def bitalino(client):
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
def HeartRateAverage(ecgMV, minHeartRate, maxHeartRate, vec=[], timeSpan, actualTime, oldTime, MinimumTime, iteration, i ,rate=[]):
    if(ecgMV <= 0.8*minHeartRate):
        MinimumTime= iteration*timeSpan
                   
    if(ecgMV >= 0.85*maxHeartRate and (iteration*timeSpan)-actualTime > 0.5 and (iteration*timeSpan)-MinimumTime < 0.35):
        i+=1
        oldTime=actualTime
        actualTime= iteration*timeSpan
        
        if i!=0:
            vec.append(1/(float(actualTime)-float(oldTime))*60)
    
        if i>3:
            #stores the actual and the previous mean value of the 4 maximums recorded
            rate[0]= rate[1]
            rate[1]=numpy.mean(vec[i-4:i-1])
 
    # Iteration is used to count the time and to calculate the heart rate
    iteration=+1
    #Time between the actual and the previous maximum
    time_delta=float(actualTime)-float(oldTime)
    return rate, time_delta


# Get Heart bpm
def GetHeartRate(client, file, ecgMV, minHeartRate, maxHeartRate, vec=[], timeSpan, actualTime, oldTime, MinimumTime, iteration, max_count, bpm_rate=[]):

    # Read ECG values for 10 x timeSpan seconds
    for x in range(1,10):
        ecgMV = bitalino(client)
        bpm_rate, time_delta = HeartRateAverage(ecgMV, minHeartRate, maxHeartRate, vec=vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, max_count, bpm_rate)
        # pace to read the sensor values 
        time.sleep(timeSpan)
    #Calculates the derivative between the actual and the previous heart rate mean value
    bpm_rate_derivative = (bpm_rate[1]-bpm_rate[0])/time_delta
    
    message = {"pulse": {"value": form(bpm_rate)} } 
    client.publish(topic="/v6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
    #message = {"bpm_rate_derivative": {"value": form(bpm_rate_derivative)} } 
    #client.publish(topic="/v6/devices/test" , payload=json.dumps(message), qos=1, retain=False )    
    file.write("%f," % bpm_rate)
    file.write("%f," % bpm_rate_derivative)


# Get Sensor Pressure 
def getPressure(client,file):
    fsr_value = []
    spi = spidev.SpiDev()
    spi.open(0,0)
    
    for i in range(5):
        fsr_value.append(readadc(i,spi))
        print fsr_value[i]
        file.write("%f," % fsr_value[i])

    # Back position
    if fsr_value[0] < 10 and fsr_value[3] < 10:
        back_position = -999
    else:
        back_position = 7.5*(fsr_value[3] - fsr_value[0])/900

    # Bottom central position
    if fsr_value[1] < 10 and fsr_value[2] < 10 and fsr_value[4] < 10:
        bottomcentral_position = -999
    else:
        # 18.6 cm ??
        bottomcentral_position = 9.3*(2*fsr_value[1] - fsr_value[2] - fsr_value[4])/1800

    # Bottom side position
    if fsr_value[2] < 10 and fsr_value[4] < 10:
        bottomside_position = -999
    else:
        # 15 cm ??
        bottomside_position = 7.5*(fsr_value[2] - fsr_value[4])/900

    back_message = {"back_position": {"value": back_position}}
    bottomcentral_message = {"bottomcentral_position": {"value": bottomcentral_position}}
    bottomside_message = {"bottomside_position": {"value": bottomside_position}}
    client.publish(topic="/v1.6/devices/test" , payload=json.dumps(back_message), qos=1, retain=False)
    client.publish(topic="/v1.6/devices/test" , payload=json.dumps(bottomcentral_message), qos=1, retain=False)
    client.publish(topic="/v1.6/devices/test" , payload=json.dumps(bottomside_message), qos=1, retain=False)


def my_state():
    global emotion_state
    while True:
        print('inserir novo estado')
        emotion_state = input()


########## MAIN ##########

if __name__ == '__main__':

    # Sensor reading flag
    sensors = True
    # data=[]
    
    # Humidity
    hum_bodyPin = 17
    hum_RoomPin = 18

    # ECG
    maxValues=[]
    minValues=[]
    ecgMV=0.0
    maxHeartRate=0
    minHeartRate=0
    vec=[]
    timeSpan=0.01
    actualTime=0
    oldTime=0
    MinimumTime=0
    iteration=0
    max_count=0
    bpm_rate={0.0, 0.0}

    # Cloud Clients
    client1 = mqtt.Client(client_id=creds1['clientId'])
    delegate1 = MqttDelegate(client1, creds1)
    ConnectToClient(client1, delegate1, creds1)
    client2 = mqtt.Client(client_id=creds2['clientId'])
    delegate2 = MqttDelegate(client2, creds2)
    ConnectToClient(client2, delegate2, creds2)
    client3 = mqtt.Client(client_id=creds3['clientId'])
    delegate3 = MqttDelegate(client3, creds3)
    ConnectToClient(client3, delegate3, creds3)
    client4 = mqtt.Client(client_id=creds4['clientId'])
    delegate4 = MqttDelegate(client4, creds4)
    ConnectToClient(client4, delegate4, creds4)
    client5 = mqtt.Client(client_id=creds5['clientId'])
    delegate5 = MqttDelegate(client5, creds5)
    ConnectToClient(client5, delegate5, creds5)
    client6 = mqtt.Client(client_id=creds6['clientId'])
    delegate6 = MqttDelegate(client6, creds6)
    ConnectToClient(client6, delegate6, creds6)

    SensorAddressSetup()

    file = open("data_sensors.txt", "a")

    emotion_state = threading.Thread(name='my_state', target=my_state)
    emotion_state.start() 

    accel_last, init_time, maxHeartRate, minHeartRate = InitializeValues(client, maxHeartRate, minHeartRate, timeSpan, ecgMV, maxValues=maxValues, minValues=minValues)
    

    ### EL CICLO ###

    while True: # replace with "sensors"
        client.loop()
        
        print "temp"
        body_temp,ctrl_temp = GetSensorTemp(client)
        # temp_media = temp_media + temp
        # data.append(temp)
        
        for k in range(1,5):
            print body_temp
            file.write("%f," % body_temp)
            print ctrl_temp
            file.write("%f," % ctrl_temp) 

            print "acceleration"
            GetAccelerometerPosition(client, accel_last, init_time, file)
            #x_media+=x_avgaccel
            #y_media+=y_avgaccel

            print "bitalino"
            GetHeartRate(client, file, ecgMV, minHeartRate, maxHeartRate, vec=vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, max_count, bpm_rate=bpm_rate)
            
            print "humidity"
            getHumidity(client, hum_bodyPin, hum_RoomPin, file)
            
            print "pressure"
            getPressure(client, file)
            
            file.write(emotion_state)
            file.write("\n")

            # DEBUG
            if DEBUG:
                print "In Iteration"
        
        # DEBUG
        if DEBUG:
            print "Out Iteration"
        
    
    # file.seek(0,0)
    file.close()
