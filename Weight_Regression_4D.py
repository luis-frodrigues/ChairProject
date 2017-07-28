import csv
from sklearn import linear_model
import numpy as np


DEBUG = True


def loadDataset(filename, trainingSet=[], nvariables=0):
    with open(filename, 'r') as csvfile:
        
        lines = csv.reader(csvfile)
        dataset = list(lines)

        if DEBUG:
            print ("dataset", dataset)
            print ("dataset length", len(dataset))
        
        for x in range(len(dataset)):
            row = []
            for y in range(nvariables):
                row.append(float(dataset[x][y]))
            trainingSet.append(row)


def main():
    # prepare data (P1, P2, P3, Weight)
    trainingSet=[]
    testSet=[]    
    loadDataset('training_Weights.txt',  trainingSet, 4)
    loadDataset('test_Weights.txt', testSet, 3)

    if DEBUG:
        print ('Train set: ' + repr(len(trainingSet)))
        print ('Test set: ' + repr(len(testSet)))

    # Create linear regression object
    regr = linear_model.LinearRegression()

    # Train the model using the training sets
    regr.fit([[i[j] for j in range(3)] for i in trainingSet], [i[3] for i in trainingSet])
    
    if True:
        print ('\nCoefficients:   ', regr.coef_)
        # Mean Squared Error test
        print ('Mean squared error: %.2f' % np.mean((regr.predict([[400.,200.,200.],[500,200,200]]) - [70,75]) ** 2))
        # Variance score: 1 is perfect prediction
        print ('Variance score: %.2f' % regr.score([[400.,200.,200.],[500,200,200]],[70,75]))

    print ('\nPrediction test: ', regr.predict(testSet))


if __name__ == '__main__':
    main()