#coding=utf-8
from collections import defaultdict as InitialDict
import logging
import sys
from heapq import *

numOfCar = 1    #每个路口每次需要计算路径的车辆数
NUMMAX = 2  #每组非优先级车需要延迟的时间单位，以优先级车出发的最大时间为一个单位

ratio = 0.3

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')


def readRoad(road_path):
    maxRoadLength = 0
    maxRoadSpeed = 0
    maxRoadLines = 0
    numOfAllLines = 0
    modifyFlag = 0

    timePlus = 50
    lengthCoff = 3
    speedCoff = 1.5
    linesCoff = 0.55
    modifyCoff = 1.0

    roadInfoListAll = []
    StartTimeList = []
    StartTimeListPriority = []
    roadIndexRoadDict = {}
    with open(road_path, 'r') as roadTxt:
        roadInfoList = roadTxt.readlines()

    roadInfoListSize = len(roadInfoList)
    for i in range(1, roadInfoListSize):
        roadInfoList[i] = roadInfoList[i].rstrip('\n')
        if roadInfoList[i] == "":
            continue
        else:
            skiproadInfo = roadInfoList[i].strip('(').rstrip(')').split(',')
            dictRoadTmp1 = {'id': int(skiproadInfo[0]), 'length': int(skiproadInfo[1]),
                            'maxSpeed': int(skiproadInfo[2]),
                            'lines': int(skiproadInfo[3]), 'begin': int(skiproadInfo[4]), 'end': int(skiproadInfo[5]),
                            'twoWay': int(skiproadInfo[6]), 'numOfUse': 0}
            numOfAllLines += dictRoadTmp1['lines']
            roadInfoListAll.append(dictRoadTmp1)
            roadIndexRoadDict[int(skiproadInfo[0])] = dictRoadTmp1

            # 找到最长的路，便于归一化
            if maxRoadLength < dictRoadTmp1['length']:
                maxRoadLength = dictRoadTmp1['length']

            if maxRoadSpeed < dictRoadTmp1['maxSpeed']:
                maxRoadSpeed = dictRoadTmp1['maxSpeed']

            if maxRoadLines < dictRoadTmp1['lines']:
                maxRoadLines = dictRoadTmp1['lines']

            if dictRoadTmp1['id'] == 5000 and modifyFlag == 0:

                timePlus = 50
                lengthCoff = 3
                speedCoff = 1.5
                linesCoff = 0.55
                modifyCoff = 1.0

                # 第一张地图非优先级超参数
                mapOneFirstOrder  = 1200
                mapOneSecondOrder = 1200
                mapOneThirdOrder  = 750
                mapOneForthOrder  = 750

                # 第一张地图优先级超参数
                mapOneFirstOrderPriority = 0
                mapOneSecondOrderPriority = 0
                mapOneThirdOrderPriority = 20
                mapOneForthOrderPriority = 20


                StartTimeList = [mapOneFirstOrder, mapOneSecondOrder,
                                 mapOneThirdOrder, mapOneForthOrder]

                StartTimeListPriority = [mapOneFirstOrderPriority, mapOneSecondOrderPriority,
                                         mapOneThirdOrderPriority, mapOneForthOrderPriority]

                modifyFlag = 1

            #第二张图
            if dictRoadTmp1['id'] == 5007 and modifyFlag == 0:
                timePlus = 50
                lengthCoff = 3
                speedCoff = 1.5
                linesCoff = 0.55
                modifyCoff = 1.0

                #第二张地图非优先级超参数
                mapTwoFirstOrder = 700
                mapTwoSecondOrder = 700
                mapTwoThirdOrder = 1050
                mapTwoForthOrder = 1050

                # 第二张地图优先级超参数
                mapTwoFirstOrderPriority = 0
                mapTwoSecondOrderPriority = 0
                mapTwoThirdOrderPriority = 20
                mapTwoForthOrderPriority = 20


                StartTimeList = [mapTwoFirstOrder, mapTwoSecondOrder,
                                 mapTwoThirdOrder, mapTwoForthOrder]

                StartTimeListPriority = [mapTwoFirstOrderPriority, mapTwoSecondOrderPriority,
                                         mapTwoThirdOrderPriority, mapTwoForthOrderPriority]

                modifyFlag = 1

            if 1 == dictRoadTmp1['twoWay']:
                dictRoadTmp2 = {'id': int(skiproadInfo[0]), 'length': int(skiproadInfo[1]),
                                'maxSpeed': int(skiproadInfo[2]),
                                'lines': int(skiproadInfo[3]), 'begin': int(skiproadInfo[5]),
                                'end': int(skiproadInfo[4]), 'twoWay': int(skiproadInfo[6]), 'numOfUse': 0}
                numOfAllLines += dictRoadTmp2['lines']
                roadInfoListAll.append(dictRoadTmp2)
                roadIndexRoadDict[int(skiproadInfo[0])] = dictRoadTmp2
    return roadInfoListAll, roadIndexRoadDict, maxRoadLength, maxRoadSpeed, maxRoadLines, \
           numOfAllLines, StartTimeList, StartTimeListPriority, timePlus, lengthCoff, speedCoff, linesCoff, modifyCoff


#利用邻接矩阵创建图
INFDEFAULT = 100000
def creatInitialGraphAndCrossToRoad(numOfCrossInfoDict, roadInfoListAll, crossIndexNumDict):
    crossToRoad = []
    graphWeight = []
    Graph = []

    crossToRoad = [[-1 for i in range(numOfCrossInfoDict + 1)] for j in range(numOfCrossInfoDict + 1)]
    for i in range(0,numOfCrossInfoDict + 1):
        for j in range(0, numOfCrossInfoDict + 1):
            crossToRoad[i][j] = 0

    graphWeight = [[-1 for i in range(numOfCrossInfoDict + 1)] for j in range(numOfCrossInfoDict + 1)]
    for i in range(0,numOfCrossInfoDict + 1):
        for j in range(0, numOfCrossInfoDict + 1):
            if i != j:
                graphWeight[i][j] = INFDEFAULT
            else:
                graphWeight[i][j] = 0

    # 对邻接矩阵和权重矩阵赋值
    roadInfoListAllSize = len(roadInfoListAll)
    for m in range(0, roadInfoListAllSize):
        indexNumx = crossIndexNumDict[roadInfoListAll[m]['begin']]
        indexNumy = crossIndexNumDict[roadInfoListAll[m]['end']]

        crossToRoad[indexNumx][indexNumy] = roadInfoListAll[m]['id']
        graphWeight[indexNumx][indexNumy] = roadInfoListAll[m]['length']

    # 创建图
    graphWeightRowSize = len(graphWeight)
    graphWeightColSize = len(graphWeight[0])
    for m in range(0, graphWeightRowSize):
        for n in range(0, graphWeightColSize):
            if m != n and graphWeight[m][n] != INFDEFAULT:
                weightMN = graphWeight[m][n]
                Graph.append((m, n, weightMN))
    return Graph, crossToRoad


def changeWeight(numOfCrossInfoDict, roadInfoListAll, roadIndexRoadDict, maxLength, maxSpeed, maxLines, crossIndexNumDict, A,
                 lengthCoff, speedCoff, linesCoff, modifyCoff):
    graphWeight = []
    Graph = []

    graphWeight = [[-1 for i in range(numOfCrossInfoDict + 1)] for j in range(numOfCrossInfoDict + 1)]
    for i in range(0,numOfCrossInfoDict + 1):
        for j in range(0, numOfCrossInfoDict + 1):
            if i != j:
                graphWeight[i][j] = INFDEFAULT
            else:
                graphWeight[i][j] = 0

    # 对邻接矩阵和权重矩阵赋值
    roadInfoListAllSize = len(roadInfoListAll)
    for m in range(0, roadInfoListAllSize):
        #print(roadInfoListAll[m]['id'])
        twoWayCoff = 1
        if roadInfoListAll[m]['twoWay'] == 0:
            twoWayCoff = 3

        B = 1.0 * roadIndexRoadDict[roadInfoListAll[m]['id']]['numOfUse'] / roadInfoListAll[m]['lines']
        lengthVar = 1.0 * roadInfoListAll[m]['length'] / maxLength
        speedVar = 1.0 * maxSpeed / roadInfoListAll[m]['maxSpeed']
        linesVar = 1.0 * maxLines / roadInfoListAll[m]['lines']
        GoldWeight = twoWayCoff * (lengthCoff * lengthVar+ speedCoff * speedVar + modifyCoff * (B / A) + linesCoff * linesVar)

        indexNumx = crossIndexNumDict[roadInfoListAll[m]['begin']]
        indexNumy = crossIndexNumDict[roadInfoListAll[m]['end']]

        graphWeight[indexNumx][indexNumy] = GoldWeight

    # 创建图
    graphWeightRowSize = len(graphWeight)
    graphWeightColSize = len(graphWeight[0])
    for m in range(0, graphWeightRowSize):
        for n in range(0, graphWeightColSize):
            if m != n and graphWeight[m][n] != INFDEFAULT:
                weightMN = graphWeight[m][n]
                Graph.append((m, n, weightMN))
    return Graph


#根据路口找路
def crossIndexRoad(crossToRoad, crossList):
    roadList = []
    rangeSize = len(crossList) - 1
    for i in range(0, rangeSize):
        cross1 = crossList[i]
        cross2 = crossList[i + 1]
        roadList.append(crossToRoad[cross1][cross2])
    return roadList


def calculateLengthAndPath(Graph, graphTmp, vBegin, vEnd, direction):
    beginID = vBegin
    endID = vEnd
    priorQueue, exitPath = [(0, beginID, ())], set()

    while priorQueue:
        (sumWeight, left, path) = heappop(priorQueue)

        if left not in exitPath:
            exitPath.add(left)
            path = (left, path)
            if left == endID:
                return sumWeight, path

            if direction == 0:
                for curWeight, right in graphTmp.get(left, ()):
                    if right not in exitPath:
                        heappush(priorQueue, (sumWeight + curWeight, right, path))
            elif direction == 1:
                for curWeight, right in graphTmp.get(left, ()):
                    if right < left:
                        continue
                    if right not in exitPath:
                        heappush(priorQueue, (sumWeight + curWeight, right, path))
            elif direction == 2:
                for curWeight, right in graphTmp.get(left, ()):
                    if right > left:
                        continue
                    if right not in exitPath:
                        heappush(priorQueue, (sumWeight + curWeight, right, path))
            elif direction == 3:
                for curWeight, right in graphTmp.get(left, ()):
                    if (right - left != 1) and (right - left != (-1 * 10)):
                        continue
                    if right not in exitPath:
                        heappush(priorQueue, (sumWeight + curWeight, right, path))
            elif direction == 4:
                for curWeight, right in graphTmp.get(left, ()):
                    if (right - left != -1) and (right - left != 10):
                        continue
                    if right not in exitPath:
                        heappush(priorQueue, (sumWeight + curWeight, right, path))
    return INFDEFAULT, []


def dijkstraCarPath(Graph, vBegin, vEnd, direction):
    GraphTmp = Graph
    beginID = vBegin
    endID = vEnd
    directionFlag = direction

    graphBinary = InitialDict(list)

    for left, right, curWeight in Graph:
        graphBinary[left].append((curWeight, right))

    calLength, drivePathQueue = calculateLengthAndPath(GraphTmp, graphBinary, beginID, endID, directionFlag)

    pathLength = -1
    drivePath = []
    drivePathQueueSize = len(drivePathQueue)
    if drivePathQueueSize > 0:
        pathLength = calLength
        left = drivePathQueue[0]
        drivePath.append(left)
        right = drivePathQueue[1]

        rightSize = len(right)
        while rightSize > 0:
            left = right[0]
            drivePath.append(left)
            right = right[1]
            rightSize = len(right)
        drivePath.reverse()

    return pathLength, drivePath


#CROSSDICT字典，key为crossid，value为道路列表
def readCrossFirst(cross_path):
    crossIDList = []
    crossInfoDictFirst = {}
    isVisit = {}
    with open(cross_path, 'r') as crossTxt:
        crossInfoList = crossTxt.readlines()
    crossInfoListSize = len(crossInfoList)
    for i in range(1, crossInfoListSize):
        crossInfoList[i] = crossInfoList[i].rstrip('\n')
        if crossInfoList[i] == "":
            continue
        else:
            skipcrossInfo = crossInfoList[i].strip('(').rstrip(')').split(',')
            id = skipcrossInfo[0]
            right = skipcrossInfo[1]
            down = skipcrossInfo[2]
            left = skipcrossInfo[3]
            up = skipcrossInfo[4]
            crossIDList.append(int(id))
            isVisit[int(id)] = False
            crossInfoDictFirst[int(id)] = [int(right), int(down), int(left), int(up)]

    return crossIDList, isVisit, crossInfoDictFirst


def customDirection(crossId, isVisit, crossInfoDictFirst, roadInfoListAll, direction=None, preCrossId=None):
    if isVisit[crossId]:
        return
    isVisit[crossId] = True
    if preCrossId is not None:
        for i in range(4):
            roadId = crossInfoDictFirst[crossId][i]
            if roadId != -1:
                roadInfoListAllSize = len(roadInfoListAll)
                for m in range(roadInfoListAllSize):
                    if roadId == roadInfoListAll[m]['id']:
                        preCrossID = roadInfoListAll[m]['begin'] if roadInfoListAll[m]['begin'] != crossId else \
                        roadInfoListAll[m]['end']
                        if preCrossID == preCrossId:
                            shift = i - (direction + 2) % 4
                            crossInfoDictFirst[crossId] = [crossInfoDictFirst[crossId][shift%4],
                                                           crossInfoDictFirst[crossId][(1+shift)%4],
                                                           crossInfoDictFirst[crossId][(2+shift)%4],
                                                           crossInfoDictFirst[crossId][(3+shift)%4]]
                            break
    for i in range(4):
        roadId = crossInfoDictFirst[crossId][i]
        if roadId != -1:
            roadInfoListAllSize = len(roadInfoListAll)
            for m in range(roadInfoListAllSize):
                if roadId == roadInfoListAll[m]['id']:
                    nextCrossId = roadInfoListAll[m]['begin'] if roadInfoListAll[m]['begin'] != crossId else \
                    roadInfoListAll[m]['end']
                    customDirection(nextCrossId, isVisit, crossInfoDictFirst, roadInfoListAll, i, crossId)


def readCrossSecond(crossIDList,  crossInfoDictFirst, isVisit,  roadInfoListAll):
    customDirection(crossIDList[0], isVisit, crossInfoDictFirst, roadInfoListAll)
    crossInfoDict = {}
    crossIndexNumDict = {}
    numCarInCurCross = {}
    i = 0
    for crossId in crossIDList:
        i += 1
        right, down, left, up = crossInfoDictFirst[crossId]
        dictTmp = {'id': i, 'roadIDRight': int(right),
                   'roadIDDown': int(down),
                   'roadIDLeft': int(left), 'roadIDUp': int(up)}
        crossIndexNumDict[crossId] = i
        crossInfoDict[dictTmp['id']] = dictTmp
        numCarInCurCross[crossId] = 0
    return crossInfoDict, numCarInCurCross, crossIndexNumDict


#读路口信息，建立索引信息
def readCross(cross_path):
    # 读取cross文件
    crossIndexNumDict = {}
    crossInfoDict = {}
    numCarInCurCross = {}
    with open(cross_path, 'r') as crossTxt:
        crossInfoList = crossTxt.readlines()
    crossInfoListSize = len(crossInfoList)
    for i in range(1, crossInfoListSize):
        crossInfoList[i] = crossInfoList[i].rstrip('\n')
        if crossInfoList[i] == "":
            continue
        else:
            skipcrossInfo = crossInfoList[i].strip('(').rstrip(')').split(',')
            dictTmp = {'id': i, 'roadIDRight': int(skipcrossInfo[1]),
                       'roadIDDown': int(skipcrossInfo[2]),
                       'roadIDLeft': int(skipcrossInfo[3]), 'roadIDUp': int(skipcrossInfo[4])}
            #两张地图路口一样，路口编号按行顺序给出且地图完整为矩形，但与序号一致或不一致
            crossIndexNumDict[int(skipcrossInfo[0])] = i

            crossInfoDict[dictTmp['id']] = dictTmp
            numCarInCurCross[int(skipcrossInfo[0])] = 0
    return crossInfoDict, numCarInCurCross, crossIndexNumDict


#读车辆信息，车辆字典里加组别，dijkstra计算总棋盘距离，然后分组
def readCar(car_path, crossIndexNumDict, numCarInCurCross,  Graph, crossInfoDict, crossToRoad):
    carInfoDict = {}
    persetCarInfoDict = {}
    crossIndexCar = InitialDict(list)
    numOfAllCarChessBoardPath = 0
    maxNumOfCarInCross = 0
    priorityCarOfPresetInfoDict = {}
    oridinaryCarOfPresetInfoDict = {}
    numCarOfPreset = 0

    with open(car_path, 'r') as carTxt:
        carInfoList = carTxt.readlines()

    carInfoListSize = len(carInfoList)
    for i in range(1, carInfoListSize):
        carInfoList[i] = carInfoList[i].rstrip('\n')
        if carInfoList[i] == "":
            continue
        else:
            skipcarInfo = carInfoList[i].strip('(').rstrip(')').split(',')
            #考虑预设车辆
            if int(skipcarInfo[6]) == 1:
                dictTmp = {'id': int(skipcarInfo[0]), 'begin': int(skipcarInfo[1]), 'end': int(skipcarInfo[2]),
                           'maxSpeed': int(skipcarInfo[3]), 'planTime': int(skipcarInfo[4]), 'group': 0,
                           'priority': int(skipcarInfo[5]), 'preset': int(skipcarInfo[6])}
                if int(skipcarInfo[5]) == 1:
                    priorityCarOfPresetInfoDict[int(skipcarInfo[0])] = dictTmp
                else:
                    oridinaryCarOfPresetInfoDict[int(skipcarInfo[0])] = dictTmp
                persetCarInfoDict[int(skipcarInfo[0])] = dictTmp
                continue
            dictTmp = {'id': int(skipcarInfo[0]), 'begin': int(skipcarInfo[1]), 'end': int(skipcarInfo[2]),
                       'maxSpeed': int(skipcarInfo[3]), 'planTime': int(skipcarInfo[4]), 'group': 0,
                       'priority': int(skipcarInfo[5]), 'preset': int(skipcarInfo[6])}

            #这里是否需要考虑预设车辆？？？
            #不需要转换为序号，直接路口编号即可
            numCarInCurCross[dictTmp['begin']] += 1

            beginID = crossIndexNumDict[dictTmp['begin']]
            endID = crossIndexNumDict[dictTmp['end']]

            #普通dijktra求出最短路径
            #roadTmpList是cross，roadPath是路
            roadTmpList = dijkstraCarPath(Graph, beginID, endID, 0)
            roadPath = crossIndexRoad(crossToRoad, roadTmpList[1])
            roadPathSize = len(roadPath)
            numRight = 0
            numDown = 0
            numLeft = 0
            numUp = 0
            for j in range(0, roadPathSize):
                roadID = roadPath[j]
                crossID = roadTmpList[1][j]
                if roadID == crossInfoDict[crossID]['roadIDRight']:
                    numRight += 1
                elif roadID == crossInfoDict[crossID]['roadIDDown']:
                    numDown += 1
                elif roadID == crossInfoDict[crossID]['roadIDLeft']:
                    numLeft += 1
                elif roadID == crossInfoDict[crossID]['roadIDUp']:
                    numUp += 1
            if numRight > numLeft:
                if numUp > numDown:
                    dictTmp['group'] = 3
                if numUp == numDown:
                    dictTmp['group'] = 1
                if numUp < numDown:
                    dictTmp['group'] = 1
            elif numRight < numLeft:
                if numUp > numDown:
                    dictTmp['group'] = 2
                if numUp == numDown:
                    dictTmp['group'] = 2
                if numUp < numDown:
                    dictTmp['group'] = 4
            else:
                if numUp > numDown:
                    dictTmp['group'] = 3
                if numUp == numDown:
                    dictTmp['group'] = 1
                if numUp < numDown:
                    dictTmp['group'] = 4

            #棋盘距离，roadPath的size
            numOfAllCarChessBoardPath += len(roadPath)

            carInfoDict[int(skipcarInfo[0])] = dictTmp
            crossIndexCar[beginID].append(dictTmp['id'])
    tmpDict = sorted(numCarInCurCross.items(),key=lambda x:x[1],reverse=True)
    maxNumOfCarInCross = tmpDict[0][1]
    return carInfoDict, numOfAllCarChessBoardPath, crossIndexCar, maxNumOfCarInCross, persetCarInfoDict, priorityCarOfPresetInfoDict, oridinaryCarOfPresetInfoDict


#处理预设车辆信息,将路径存入各自的字典
#包括车辆ID和预设路径,将预设车辆中的优先级车辆和非优先级车辆分开，并按照路径的长度从大到小排序
def readPreset(preset_answer_path, persetCarInfoDict):
    presetPathDict = {}
    priorityCarPathOfPresetDictTmp = {}
    ordinaryCarPathOfPresetDictTmp = {}
    priorityCarOfPresetID = []
    ordinaryCarOfPresetID = []
    with open(preset_answer_path, 'r') as presetCarTxt:
        presetCarInfoList = presetCarTxt.readlines()

    presetCarInfoListSize = len(presetCarInfoList)
    for i in range(1, presetCarInfoListSize):
        presetCarInfoList[i] = presetCarInfoList[i].rstrip('\n')
        if presetCarInfoList[i] == "":
            continue
        else:
            skippersetCarInfo = presetCarInfoList[i].strip('(').rstrip(')').split(',')
            skippersetCarInfoSize = len(skippersetCarInfo)
            presetCarRoadPath = []
            for i in range(2, skippersetCarInfoSize):
                presetCarRoadPath.append(int(skippersetCarInfo[i]))
            dictTmp = {"pathSize": len(presetCarRoadPath), "path": presetCarRoadPath, "startTime": int(skippersetCarInfo[1])}
            if persetCarInfoDict[int(skippersetCarInfo[0])]['priority'] == 1:
                priorityCarPathOfPresetDictTmp[int(skippersetCarInfo[0])] = dictTmp
            else:
                ordinaryCarPathOfPresetDictTmp[int(skippersetCarInfo[0])] = dictTmp
            presetPathDict[int(skippersetCarInfo[0])] = dictTmp
            # print(int(skippersetCarInfo[0])," : ", presetCarRoadPath )

    priorityCarPathOfPresetDict = sorted(priorityCarPathOfPresetDictTmp.items(),key=lambda x:x[1]['pathSize'],
                                         reverse=True)
    ordinaryCarPathOfPresetDict = sorted(ordinaryCarPathOfPresetDictTmp.items(), key=lambda x: x[1]['pathSize'],
                                         reverse=True)

    #将按照路径长短排序后的车辆ID分别存入优先车队和普通车队
    for i in range(0, len(priorityCarPathOfPresetDict)):
        priorityCarOfPresetID.append(priorityCarPathOfPresetDict[i][0])
    for i in range(0, len(ordinaryCarPathOfPresetDict)):
        ordinaryCarOfPresetID.append(ordinaryCarPathOfPresetDict[i][0])

    return presetPathDict, priorityCarOfPresetID, ordinaryCarOfPresetID

#从预设车辆中选择30%的车辆自己规划路线,三个思路
#（1）选择优先级高的车辆，
#（2）选择出发时间靠后的车辆，
#（3）选择路线长的车辆
def selectCar(presetPathDict, persetCarInfoDict, priorityCarOfPresetID, ordinaryCarOfPresetID, carInfoDict,
                 numOfAllCarChessBoardPath, crossIndexCar, crossIndexNumDict, numCarInCurCross, Graph, crossInfoDict,
                 crossToRoad, maxNumOfCarInCross):
    print(numOfAllCarChessBoardPath, " ", maxNumOfCarInCross)
    selectCarID = []
    priorityCarOfPresetIDUpdate = []
    ordinaryCarOfPresetIDUpdate = []
    numOfSelectcar = int(len(persetCarInfoDict) * ratio)
    #若选择的车辆数目小于预设车辆中的优先级车来嗯
    if len(priorityCarOfPresetID) >= numOfSelectcar:
        priorityCarOfPresetIDUpdate = priorityCarOfPresetID[:]
        for i in range(0, numOfSelectcar):
            selectCarID.append(priorityCarOfPresetID[i])
            priorityCarOfPresetIDUpdate.remove(priorityCarOfPresetID[i])   #这里删除元素对不对？？

    elif len(priorityCarOfPresetID) < numOfSelectcar:
        selectCarID = priorityCarOfPresetID[:]
        ordinaryCarOfPresetIDUpdate = ordinaryCarOfPresetID[:]
        numOfReserveSelectCar = numOfSelectcar - len(priorityCarOfPresetID)
        for i in range(0, numOfReserveSelectCar):
            selectCarID.append(ordinaryCarOfPresetID[i])
            ordinaryCarOfPresetIDUpdate.remove(ordinaryCarOfPresetID[i])

    print(len(selectCarID), " ", len(persetCarInfoDict) * ratio)

    #选择完车辆后，将选择出来的车辆更新进普通车队，更新预设车队
    for i in range(0, len(selectCarID)):
        del presetPathDict[selectCarID[i]]

        dictTmp = persetCarInfoDict[selectCarID[i]]
        numCarInCurCross[dictTmp['begin']] += 1

        beginID = crossIndexNumDict[dictTmp['begin']]
        endID = crossIndexNumDict[dictTmp['end']]

        # 普通dijktra求出最短路径
        # roadTmpList是cross，roadPath是路
        roadTmpList = dijkstraCarPath(Graph, beginID, endID, 0)
        roadPath = crossIndexRoad(crossToRoad, roadTmpList[1])
        roadPathSize = len(roadPath)
        numRight = 0
        numDown = 0
        numLeft = 0
        numUp = 0
        for j in range(0, roadPathSize):
            roadID = roadPath[j]
            crossID = roadTmpList[1][j]
            if roadID == crossInfoDict[crossID]['roadIDRight']:
                numRight += 1
            elif roadID == crossInfoDict[crossID]['roadIDDown']:
                numDown += 1
            elif roadID == crossInfoDict[crossID]['roadIDLeft']:
                numLeft += 1
            elif roadID == crossInfoDict[crossID]['roadIDUp']:
                numUp += 1
        if numRight > numLeft:
            if numUp > numDown:
                dictTmp['group'] = 3
            if numUp == numDown:
                dictTmp['group'] = 1
            if numUp < numDown:
                dictTmp['group'] = 1
        elif numRight < numLeft:
            if numUp > numDown:
                dictTmp['group'] = 2
            if numUp == numDown:
                dictTmp['group'] = 2
            if numUp < numDown:
                dictTmp['group'] = 4
        else:
            if numUp > numDown:
                dictTmp['group'] = 3
            if numUp == numDown:
                dictTmp['group'] = 1
            if numUp < numDown:
                dictTmp['group'] = 4

        # 棋盘距离，roadPath的size
        numOfAllCarChessBoardPath += len(roadPath)

        crossIndexCar[beginID].append(dictTmp['id'])
        carInfoDict[selectCarID[i]] = persetCarInfoDict[selectCarID[i]]
    tmpDict = sorted(numCarInCurCross.items(), key=lambda x: x[1], reverse=True)
    maxNumOfCarInCross = tmpDict[0][1]
    return numOfAllCarChessBoardPath, maxNumOfCarInCross


#更新每条预设路的使用次数
def presetChangeNumuse(presetPathDict, roadIndexRoadDict):
    presetPathDictKeys = presetPathDict.keys()
    sortPresetPathDict = sorted(presetPathDictKeys)

    for mKeys in sortPresetPathDict:
        mValues = presetPathDict[mKeys]['path']
        # print(mValues)
        for k in mValues:
            # print(roadIndexRoadDict[k]['numOfUse'])
            roadIndexRoadDict[k]['numOfUse'] = roadIndexRoadDict[k]['numOfUse'] + 1


def writeAnswer(answer_path, answerDict):
    with open(answer_path, 'w') as answerTxt:
        title = '#(carId,StartTime,RoadId...)' + '\n'
        answerTxt.write(title)
        answerKeys = answerDict.keys()
        #顺序输出
        sortAnswer = sorted(answerKeys)

        sortAnswerSize = len(sortAnswer)
        for m in range(0, sortAnswerSize):
            text = sortAnswer[m]
            writeTxt = ''
            answerDictTextSize = len(answerDict[text])
            for n in range(0, answerDictTextSize):
                specificText = answerDict[text][n]
                if 0 == n:
                    writeTxt += '(' + str(specificText)
                else:
                    writeTxt += ', ' + str(specificText)

            writeTxt += ')' + '\n'
            answerTxt.write(writeTxt)

        answerTxt.close()


def DynamicDrivePath(numOfCrossInfoDict, roadInfoListAll,  maxRoadLength, maxRoadSpeed,
                    maxRoadLines, crossIndexNumDict, A, carInfoDict, maxNumOfCarInCross, numOfCar,
                    crossIndexCar, crossToRoad, roadIndexRoadDict, lengthCoff, speedCoff, linesCoff, modifyCoff):
    # 初始化最终出发时间
    allCarPath = []
    for i in range(0, 4):
        allCarPath.append(InitialDict(list))

    # 循环更新路径的次数
    for i in range(0, maxNumOfCarInCross // numOfCar):

        Graph = changeWeight(numOfCrossInfoDict, roadInfoListAll, roadIndexRoadDict, maxRoadLength, maxRoadSpeed,
                             maxRoadLines, crossIndexNumDict, A, lengthCoff, speedCoff, linesCoff, modifyCoff)
        # 循环路口
        for cross in crossIndexCar.keys():
            for j in range(numOfCar):
                if j < len(crossIndexCar[cross]):
                    #beginID = crossIndexNumDict[cross]
                    beginID = cross
                    endID = crossIndexNumDict[carInfoDict[crossIndexCar[cross][j]]['end']]
                    ID = carInfoDict[crossIndexCar[cross][j]]['id']
                    planTime = carInfoDict[crossIndexCar[cross][j]]['planTime']
                    maxspeed = carInfoDict[crossIndexCar[cross][j]]['maxSpeed']
                    group = carInfoDict[crossIndexCar[cross][j]]['group']
                    priority = carInfoDict[crossIndexCar[cross][j]]['priority']
                    crossIndexCar[cross].pop(j)

                    roadTmpList = dijkstraCarPath(Graph, beginID, endID, 0)

                    roadPath = crossIndexRoad(crossToRoad, roadTmpList[1])
                    for k in roadPath:
                        roadIndexRoadDict[k]['numOfUse'] = roadIndexRoadDict[k]['numOfUse'] + 1
                    dictTmp = {'id': ID, 'start': planTime, 'drivePath': roadPath, 'priority': priority}
                    allCarPath[group - 1][maxspeed].append(dictTmp)

    return allCarPath


# 速度降序排列，改变出发时间
def changeTimePlus(allCarPath, StartTimeList, StartTimeListPriority, timePlus):
    answerDict = InitialDict(list)
    allCarPathSize = len(allCarPath)
    #修改每组车辆的时间
    for m in range(0, allCarPathSize):
        if len(allCarPath[m]) != 0:
            # speedList = {v1:{}, v2:{}...}
            # 升序排列，再反转
            allCarPathMKeys = allCarPath[m].keys()
            speedList = sorted(allCarPathMKeys)
            speedList.reverse()
            speedListSize = len(speedList)
            for num in range(0, speedListSize):
                textPathDict = speedList[num]
                extraTime = num * timePlus
                allCarPathtextPathDictSize = len(allCarPath[m][textPathDict])
                for text in range(0, allCarPathtextPathDictSize):
                    ID = allCarPath[m][textPathDict][text]['id']
                    whichCarID = allCarPath[m][textPathDict][text]['id']
                    priority = allCarPath[m][textPathDict][text]['priority']

                    #优先级高的车辆自己分组发车，与非优先级的车分开
                    if priority == 1:
                        startTime = allCarPath[m][textPathDict][text]['start'] + extraTime + StartTimeListPriority[m]
                    else:
                        startTime = allCarPath[m][textPathDict][text]['start'] + extraTime + StartTimeList[m]

                    answerDict[whichCarID].append(ID)
                    answerDict[whichCarID].append(startTime)

                    drivePathSize = len(allCarPath[m][textPathDict][text]['drivePath'])
                    for path in range(0, drivePathSize):
                        whichRoad = allCarPath[m][textPathDict][text]['drivePath'][path]
                        answerDict[whichCarID].append(whichRoad)
    return answerDict


# #将优先级车辆分组发车(或者一起发车)，非优先级车辆分组发车(或一起发车)
# def

def main():
    if len(sys.argv) != 6:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    preset_answer_path = sys.argv[4]
    answer_path = sys.argv[5]


    # car_path = '/home/qgy/HW/SDKHW/SDK/2-map-training-1/car.txt'
    # road_path = '/home/qgy/HW/SDKHW/SDK/2-map-training-1/road.txt'
    # cross_path = '/home/qgy/HW/SDKHW/SDK/2-map-training-1/cross.txt'
    # answer_path = '/home/qgy/HW/SDKHW/SDK/2-map-training-1/answer.txt'
    # preset_answer_path = '/home/qgy/HW/SDKHW/SDK/2-map-training-1/presetAnswer.txt'


    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("preset_answer_path is %s" % (preset_answer_path))
    logging.info("answer_path is %s" % (answer_path))

    '''
    读取文件
    '''

    # 读取road文件
    roadInfoListAll, roadIndexRoadDict, maxRoadLength, maxRoadSpeed, maxRoadLines, numOfAllLines, \
    StartTimeList, StartTimeListPriority, timePlus, lengthCoff, speedCoff, linesCoff, \
    modifyCoff = readRoad(road_path)

    #读取cross文件
    CROSSNAMESPACE, visitDone, CROSSDICT = readCrossFirst(cross_path)
    crossInfoDict, numCarInCurCross, crossIndexNumDict = readCrossSecond(CROSSNAMESPACE, CROSSDICT, visitDone,  roadInfoListAll)


    # 创建有向图
    numOfCrossInfoDict = len(crossInfoDict)
    graphInitial, crossToRoad = creatInitialGraphAndCrossToRoad(numOfCrossInfoDict, roadInfoListAll, crossIndexNumDict)

    # 读取car文件
    carInfoDict, numOfAllCarChessBoardPath, crossIndexCar, maxNumOfCarInCross, persetCarInfoDict,\
    priorityCarOfPresetInfoDict, oridinaryCarOfPresetInfoDict = readCar(car_path, crossIndexNumDict, numCarInCurCross,
                                                                   graphInitial, crossInfoDict, crossToRoad)


    #读取预设车辆文件
    presetPathDict, priorityCarOfPresetID, ordinaryCarOfPresetID = readPreset(preset_answer_path, persetCarInfoDict)

    numOfAllCarChessBoardPath, maxNumOfCarInCross = selectCar(presetPathDict, persetCarInfoDict, priorityCarOfPresetID,
                                                              ordinaryCarOfPresetID, carInfoDict,numOfAllCarChessBoardPath,
                                                              crossIndexCar, crossIndexNumDict, numCarInCurCross,
                                                              graphInitial, crossInfoDict,crossToRoad, maxNumOfCarInCross)

    A = 1.0 * numOfAllCarChessBoardPath / numOfAllLines

    #更新每条路的使用次数
    presetChangeNumuse(presetPathDict, roadIndexRoadDict)


    '''
     处理规划路径，修改出发时间
    '''
    #搜寻最短路径
    allCarPath = DynamicDrivePath(numOfCrossInfoDict, roadInfoListAll,  maxRoadLength, maxRoadSpeed, maxRoadLines,
                                  crossIndexNumDict, A, carInfoDict, maxNumOfCarInCross, numOfCar, crossIndexCar,
                                  crossToRoad, roadIndexRoadDict, lengthCoff, speedCoff, linesCoff, modifyCoff)

    #修改出发时间
    answerDict = changeTimePlus(allCarPath,StartTimeList, StartTimeListPriority, timePlus)

    #统计每组的车辆
    numOfRightDown = 0
    numOfLeftUp = 0
    numOfRightUp = 0
    numOfLeftDown = 0
    for kv in carInfoDict.items():
        if kv[1]['group'] == 1:
            numOfRightDown += 1
        if kv[1]['group'] == 2:
            numOfLeftUp += 1
        if kv[1]['group'] == 3:
            numOfRightUp += 1
        if kv[1]['group'] == 4:
            numOfLeftDown += 1
    print(numOfRightDown)
    print(numOfLeftUp)
    print(numOfRightUp)
    print(numOfLeftDown)


    '''
    写入文件
    '''
    # to write output file
    writeAnswer(answer_path, answerDict)


if __name__ == "__main__":
    main()