from Greedy_trangulation_test import build_point_list
import math
import numpy as np
def build_point_face_list(filename ):
    f = open(filename,'rt')
    point_list = []
    face_list = []
    for line in f:
        if line[:2] == 'v ':
            line_split = line.strip('\n').split(' ')
            point_list.append([float(line_split[1]),float(line_split[2]),float(line_split[3])])
        if line[0] == 'f':
            line_split = line.strip('\n').split(' ')
            face_list.append([int(line_split[1].split('//')[0])-1,int(line_split[2].split('//')[0])-1,int(line_split[3].split('//')[0])-1])

    return point_list,face_list



def cntDistance(point1, point2):
    return np.sqrt(np.sum(np.square(np.array(point1)-np.array(point2))))
def main():
    filename = 'r4.obj'
    pointList,faceList = build_point_face_list(filename)
    minDistance = math.inf;
    maxDistance = 0
    for face in faceList:
        edge1_len = cntDistance(pointList[face[0]], pointList[face[1]])
        edge2_len = cntDistance(pointList[face[0]], pointList[face[2]])
        edge3_len = cntDistance(pointList[face[2]], pointList[face[1]])
        minDistance = min([edge1_len,edge2_len,edge3_len,minDistance])
        maxDistance = max([edge1_len, edge2_len, edge3_len, minDistance])
    print("Minial distance in edge of face is " , minDistance)
    print("Maximal distance in edge of face is " , maxDistance)
if __name__ == '__main__':
    main()