import json
import time
#import context
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import Adafruit_TMP.TMP006 as TMP006
import RPi.GPIO as GPIO
import spidev
import Adafruit_DHT
import lsm303d
import threading
import csv
import math
import numpy

DEBUG = True

# DO NOT try to set values under 200 ms or the server will kick you out
publishing_period = 1000

# Temperature
controltemp_Sensor = TMP006.TMP006()
bodytemp_Sensor = TMP006.TMP006(address=0x44)
ctrltemp_flag = 0
diftemp_flag = False
flag_hum= False

# Humidity
hum_sensor = Adafruit_DHT.DHT11

# Accelerometer
accel_Sensor = lsm303d.lsm303d() # 0x1D


estado= "normal"

# mqtt credentials. Also make sure to change device_id at bottom of page
creds = {
    'clientId': '<ana.silvestre>',
    'user':     '<wZLdw10g2j2thyC9mZvaXrk5xKUWGr>',
    'password': '<>',
    'topic':    '</v1.6/devices/hp_p>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds2 = {
    'clientId': '<pressure_b>',
    'user':     '<lRel1Ay8b5uI8pvruM6uHadB1v8Baq>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds3 = {
    'clientId': '<pressure_d>',
    'user':     '<4iJNU25nVLAeNUQrVU1R34ZXFKpdxi>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds4 = {
    'clientId': '<hum.temp>',
    'user':     '<l6ZnkD9p0CJ1OfNk2pw4gIi8sYK3tm>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds5 = {
    'clientId': '<accel>',
    'user':     '<TjiSzIhMTa5EitFgugmG9ENttYCIOD>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

creds6 = {
    'clientId': '<bitpm>',
    'user':     '<zxNGSEdwIWd8JZRCcsmfd6kOzWdXVR>',
    'password': '<>',
    'topic':    '</v1.6/devices/pressure>',
    'server':   'mqtt.things.ubidots.com',
    'port':     1883
}

#Gives the max and min values that will be used for the tresholds
def HeartRate(heartValue=0, TimeEnd=False, maxValues=[], minValues=[]):   
    minHeartRate=0
    maxHeartRate=0
    
    if not TimeEnd :
        maxValues.append(float(heartValue)**3)
    else:
        maxValues.sort(reverse=True)
        minValues= maxValues[-4:-1]        
        maxValues=maxValues[:4]
        #Calculates the mean values of the vector for the max and min values
        minHeartRate=numpy.mean(minValues)
        maxHeartRate=numpy.mean(maxValues)
        return maxHeartRate,minHeartRate;    
    return maxHeartRate, minHeartRate;

#Calculates the mean value of heart rate from the last 4 values recorded
def hearRateAverage(ecgMV, minHeartRate, maxHeartRate, vec=[], timeSpan, actualTime, oldTime, MinimumTime, iteration, i ,rate):
    if(ecgMV <= 0.8*minHeartRate):
        MinimumTime= iteration*timeSpan
                   
    if(ecgMV >= 0.85*maxHeartRate and (iteration*timeSpan)-actualTime > 0.5 and (iteration*timeSpan)-MinimumTime < 0.35):
        i+=1
        oldTime=actualTime
        actualTime= iteration*timeSpan
        
        if i!=0:
            vec.append(1/(float(actualTime)-float(oldTime))*60)
    
        if(i>3):
            rate=numpy.mean(vec[i-4:i-1])
 
    #iteration is used to count the time and to calculate the heart rate
    iteration=+1
    return rate;

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


# Sensor address setup
def SensorAddressSetup():
    controltemp_Sensor.begin()
    bodytemp_Sensor.begin(samplerate = TMP006.CFG_8SAMPLE)


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
    client.publish(topic="/v1.6/devices/test" , payload=json.dumps({"body_temperature": {"value":0}}), qos=1, retain=False )

    # Accel
    accel_last = accel_Sensor.getRealAccel()
    init_time = time.clock()

    return accel_last, init_time


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
        #para escrever no ficheiro
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
def getHumidity(client, bodyPin, RoomPin, file):
    
    global flag_hum
    bodyHumidity, bodyTemperature = Adafruit_DHT.read_retry(hum_sensor, bodyPin)
    roomHumidity, bodyTemperature2 = Adafruit_DHT.read_retry(hum_sensor, RoomPin)
    
    deltahum=bodyHumidity-roomHumidity

    if deltahum < 5 and flag_hum:
        message = {"dif_Humidity": {"value": 0} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        #para escrever no ficheiro
        bodyHumidity=0
        flag_hum=False
        
    elif deltahum>5:
        if not flag_hum:
            message = {"dif_Humidity": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_hum = True            
            
        message = {"dif_Humidity": {"value": form(deltahum)}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )               
    
    file.write("%f " % bodyHumidity)
    file.write("%f " % roomHumidity)

    return bodyHumidity, roomHumidity


# Get Accelerometer Position
def GetAccelerometerPosition(client, accel_last, init_time, file):

    accel = accel_Sensor.getRealAccel()
    end_time = time.clock()
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
    
    if (accel[0]<0.1 or accel[0]>0.1) and flag_acc:
        message = {"x_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        #para escrever no ficheiro
        accel[0]=0
        flag_acc=False        
    elif accel[0]>0.1 or accel[0]<0.1:
        if not flag_acc:
            message = {"x_accel": {"value": 0}}
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
            flag_acc = True

        message = {"x_accel": {"value": form(accel[0])} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )

    if (accel[0]<0.1 or accel[0]>0.1) and flag_acc:
        message = {"x_accel": {"value": 0}}
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False ) 
        #para escrever no ficheiro
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
        #para escrever no ficheiro
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

    file.write("%f " % accel_squared)
    file.write("%f" % jerk)

    time.sleep(0.2)


# Get Bitalino bpm
def bitalino(client):
    spi = spidev.SpiDev()
    spi.open(0,0)
    bitalino = readadc(5,spi)
    ecgV = (bitalino*3.3/2**10-3.3/2)/1100 #VALUE IN VOLTS
    ecgMV = ecgV*1000 #value in mvolts
    
    return ecgMV


# Get Sensor Pressure 
def getPressure(client,file):
    fsr_value = []
    spi = spidev.SpiDev()
    spi.open(0,0)
    
    for i in range(5):
        fsr_value.append(readadc(i,spi))
        print fsr_value[i]
        file.write("%f " % fsr_value[i])
        message = {"pressure"+str(i): {"value": fsr_value[i]} }
        client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False)     
        

def my_state():
    global estado
    while True:
        print('inserir novo estado')
        estado = input()
    

########## MAIN ##########

if __name__ == '__main__':

    sensors = True
    bodyPin = 17
    RoomPin = 18    
    temp_media =0
    SensorAddressSetup()
    data=[]
    timeSpan=0.01
    maxHeartRate=0
    minHeartRate=0
    maxValues=[]
    minValues=[]
    MinimumTime=0
    iteration=0
    actualTime=0
    oldTime=0
    bpm_rate=0.0
    i=0
    ecgMV=0.0
    vec=[]    
    
    state = threading.Thread(name='my_state', target=my_state)
    state.start()
    
    client = mqtt.Client(client_id=creds['clientId'])
    delegate = MqttDelegate(client, creds)
    ConnectToClient(client, delegate, creds)

    accel_last, init_time = InitializeValues(client)
    file = open("data_sensors.txt", "a") 
    
    #The first 5.2 seconds are needed to create a threshold for the max points
    #to calculate then the heart rate
    start = time.time()
    while (time.time()-start)<5.2:
        for i in range(1,5):
            ecgMV = bitalino(client)
            maxHeartRate, minHeartRate= HeartRate(ecgMV, False, maxValues=maxValues, minValues=minValues)
            time.sleep(timeSpan)
    
    #Make the mean of values to get the threshold of max and min values
    maxHeartRate, minHeartRate= HeartRate(ecgMV, True, maxValues=maxValues, minValues=minValues)    
    
    while True:
        client.loop()
        
        print "temp"
        body_temp,ctrl_temp = GetSensorTemp(client)
        #faz a media das diferencas da temperatura
        temp_media = temp_media + temp
        data.append(temp)
        
        for k in range(1,5):
            #Read ECG values for 10 x timeSpan seconds
            for x in range(1,10):           
                ecgMV = bitalino(client)
                bpm_rate = hearRateAverage(ecgMV, minHeartRate, maxHeartRate, vec=vec, timeSpan, actualTime, oldTime, MinimumTime, iteration, i ,bpm_rate)
                #pace to read the sensor values 
                time.sleep(timeSpan)           
                    
            print body_temp
            print ctrl_temp   
            
            file.write(estado)
            file.write("\n")            
             
            file.write("%f " % body_temp)
            file.write("%f " % ctrl_temp)            
            GetAccelerometerPosition(client, accel_last, init_time, file)
            
            print "bitalino"
            file.write("%f," % bpm_rate)
            message = {"pulse": {"value": form(bpm_rate)} } 
            client.publish(topic="/v1.6/devices/test" , payload=json.dumps(message), qos=1, retain=False )
             
            #x_media+=x_avgaccel
            #y_media+=y_avgaccel

            print "humidity"
            bodyHumidity, rooomHumidity = getHumidity(client, bodyPin, RoomPin, file)
            
            print "pressure"
            getPressure(client, file)
            
            file.write(estado)
            file.write("\n")
        
        # DEBUG
        if DEBUG:
            print "Iteration"
        
    # print(len(file))    
    # temp_media=temp_media/len(file)
    # x_global=x_media/len(file)
    # y_global=y_media/len(file)
    
    file.seek(0,0)
    # file.write(file,[temp_media,x_media,y_media])
    file.close()