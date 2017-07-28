import csv
import random
import math
import numpy

def loadDataset(filename, trainingSet=[], nvariables=0):
    with open(filename, 'r') as csvfile:
        
        lines = csv.reader(csvfile)
        dataset = list(lines)
        #print("dataset",dataset)
        print("dataset legth",len(dataset))
        
        for x in range(int(len(dataset))):
            trainingSet.append(dataset[x])
        #print("dataset",dataset)

def HeartRate2(heartValue=0, timeSpan=0.1, maxValues2=[], maxHeartRate=0, n_points=0, minValues=[]):  
    
    points_needed=6/timeSpan
    minHeartRate=0
    if n_points< points_needed :
        maxValues2.append(float(heartValue)**3)
        n_points+1
    elif n_points == points_needed:
        maxValues2.sort(reverse=True)
        minValues= maxValues2[-4:-1]        
        maxValues2=maxValues2[:4]
        print(minValues)
        minHeartRate=numpy.mean(minValues)
        maxHeartRate=numpy.mean(maxValues2)
        return False, maxHeartRate,minHeartRate ;    
    return True, maxHeartRate, minHeartRate; 
     

def HeartRate1(heartValue=0, timeSpan=0.1, maxValues=[], maxHeartRate=0,n_points=0,minValues1=[]) :  
    
    points_needed=int(6/timeSpan)
    minHeartRate=0
    
    if n_points< points_needed:
        maxValues.append(float(heartValue))
        n_points+1
    elif n_points == points_needed  :
        maxValues.sort(reverse=True)
        minValues1= maxValues[-4:-1]
        maxValues=maxValues[:4]        
        print(maxValues)
        print(minValues1)
        minHeartRate=numpy.mean(minValues1)
        maxHeartRate=numpy.mean(maxValues)
        print("maxHeartRate",maxHeartRate)
        return False, maxHeartRate,minHeartRate ;    
    return True, maxHeartRate, minHeartRate; 



def main ():
    #VARIABLE INITIALIZATION
    trainingSet=[] #measures of ECG in txt
    flag= True
    flag2= True
    iteration=0
    maxValues=[]
    maxValues2=[]
    minValues=[]
    minValues1=[]
    maxHeartRate=0
    minHeartRate=0.1
    actualTime=0
    oldTime=0
    MinimumTime=0
    timeSpan=float(raw_input("What is the time span?\n"))
    rate=0.0                        #tempo cloud bitalino
    
    #Read data from txt
    nvariables = int(raw_input("How many variables?\n"))
    loadDataset('pulse.txt',  trainingSet, nvariables)   
    
    #Read for five second#
    #no codigo principal mudar para if #
    while flag:
        #print"trainingSet[iteration][0]", trainingSet[iteration][0]
        flag, maxHeartRate, minHeartRate= HeartRate1(trainingSet[iteration][0], timeSpan, maxValues, maxHeartRate, n_points = iteration, minValues1=minValues1)
        iteration=iteration+1
    
    #initializate variable to count the time
    iteration=0
    
    vec=[]
    i=-1
    ians=0
    
    #iterate the trainingSet and print the heart rate
    for x in trainingSet:
        if((float(x[0]))<=0.8):
            MinimumTime= iteration*timeSpan
                       
        if((float(x[0]))>= 0.85*maxHeartRate and (iteration*timeSpan)-actualTime > 0.5 and (iteration*timeSpan)-MinimumTime < 0.35):
            i+=1
            oldTime=actualTime
            actualTime= iteration*timeSpan
            
            if i!=0:
                vec.append(1/(float(actualTime)-float(oldTime))*60)
        
            if(i>3):
                #i=0
                rate=numpy.mean(vec[i-4:i-1])
                print("Hearth Rate2: %f\n" % rate)
        
        iteration=iteration+1
        file.write("%f " % rate)
        message = {"pulse": {"value": form(rate)} }        
    print("cheguei aqui\n")   
    
    
    
    # ####### Second Method ######## #   
    #Read for five second
    #initializate variable to count the time    
    iteration=0
  
    actualTime=0
    
    while flag2:
                                              
        flag2, maxHeartRate, minHeartRate= HeartRate2(trainingSet[iteration][0], timeSpan, maxValues2, maxHeartRate, n_points = iteration, minValues=minValues)
        iteration=iteration+1
    
    #initializate variable to count the time
    iteration=0
    actualTime=0
    print "minValues",minHeartRate
    
    #iterate the trainingSet and print the heart rate
    for y in trainingSet:
        if((float(y[0])**3)<=0.5):
            MinimumTime= iteration*timeSpan
                       
        if((float(y[0])**3)>= 0.6*maxHeartRate and (iteration*timeSpan)-actualTime > 0.5 and (iteration*timeSpan)-MinimumTime < 0.35):
            oldTime=actualTime
            actualTime= iteration*timeSpan
            print"oldTime:", oldTime , "actualtime:", actualTime
            if(oldTime!=0):
                rate=float(1/(actualTime-oldTime))*60
                print("Hearth Rate2: %f\n" % rate)
        iteration=iteration+1    
main()