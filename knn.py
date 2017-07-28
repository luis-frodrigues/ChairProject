# Example of kNN implemented from Scratch in Python

import csv
import random
import math
import operator
import math

def loadDataset(filename, trainingSet=[], nvariables=0):
    with open(filename, 'r') as csvfile:
        
        lines = csv.reader(csvfile)
        dataset = list(lines)
        print("dataset",dataset)
        print("dataset legth",len(dataset))
        
        for x in range(int(len(dataset))):
            for y in range(nvariables):
                dataset[x][y] = float(dataset[x][y])
            trainingSet.append(dataset[x])
          
def euclideanDistance(instance1, instance2, length):
    distance = 0
    for x in range(length):
        distance += pow((instance1[x] - instance2[x]), 2)
    return distance

def getNeighbors(trainingSet, testInstance, k):
    distances = []
    #print("testInstance", testInstance)
    #print("trainingInstance", trainingSet)
    length = len(testInstance)-1
    for x in range(len(trainingSet)):
        dist = euclideanDistance(testInstance, trainingSet[x], length)
        distances.append((trainingSet[x], dist))
    distances.sort(key=operator.itemgetter(1))
    neighbors = []
   
    for x in range(k):
        neighbors.append(distances[x][0])
    return neighbors

def getResponse(neighbors):
    classVotes = {}
    for x in range(len(neighbors)):
        response = neighbors[x][-1]
        #print("response", response)
        if response in classVotes:
            classVotes[response] += 1
        else:
            classVotes[response] = 1
   
    sortedVotes = sorted(classVotes.items(), key=operator.itemgetter(1), reverse=True)
    print("sorted",sortedVotes[0][0])
    return sortedVotes[0][0]

def odd(n):
    odd= int(math.sqrt(n))
    if odd%2==0:
        return odd+1
    return odd
    
def main():
    # prepare data
    trainingSet=[]
    testSet=[]
    nvariables = int(input("How many variables?\n"))
    loadDataset('Iris.txt',  trainingSet, nvariables)
    loadDataset('Test.txt', testSet, nvariables)
    #print ('Train set: ' + repr(len(trainingSet)))
    #print ('Test set: ' + repr(len(testSet)))  
    
    #Calculates odd K 
    k = odd(len(trainingSet))
    
    for x in range(len(testSet)):
        neighbors = getNeighbors(trainingSet, testSet[x], k)
        result = getResponse(neighbors)
        print(' actual=' + repr(testSet[x][-1]))
   
main()