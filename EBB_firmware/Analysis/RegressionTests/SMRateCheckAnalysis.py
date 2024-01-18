import sys

# SM,25090,-35,53415,119827,71896,40.00016691,182874195,182846806,0.01497696

dataFile = open(sys.argv[1], "r")

maxError = 0.0
errorLine = ""
lineCount = 0
maxDuration = 0
minStep = 0
maxStep = 0

print("Starting analysis of " + sys.argv[1])

for dataLine in dataFile:
    lineCount = lineCount + 1
    dataList = dataLine.split(',')
    if len(dataList) >= 9:
        if int(dataList[1]) > maxDuration:
            maxDuration = int(dataList[1])
        if int(dataList[2]) > maxStep:
            maxStep = int(dataList[2])
        if int(dataList[3]) > maxStep:
            maxStep = int(dataList[3])
        if int(dataList[2]) < minStep:
            minStep = int(dataList[2])
        if int(dataList[3]) < minStep:
            minStep = int(dataList[3])
        if abs(float(dataList[6])) > maxError:
            maxError = abs(float(dataList[6]))
            errorLine = dataLine
        if abs(float(dataList[9])) > maxError:
            maxError = abs(float(dataList[9]))
            errorLine = dataLine

print("Read in " + str(lineCount) + " lines with " + str(maxError) + " error, maxDuration= " + str(maxDuration), " minStep= " + str(minStep) + " maxStep= " + str(maxStep))
print(errorLine)

dataFile.close()
