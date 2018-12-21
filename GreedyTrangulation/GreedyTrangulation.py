import argparse
import math
import queue
import numpy as np
import kdtree
from scipy.spatial import Delaunay
#Create KD tree

# we assume that in the point cloud, every point has its corresponding  normal vector
class Point3D:
    def __init__(self, pointInfo):
        self.position= np.array([pointInfo[0],pointInfo[1],pointInfo[2]])
        self.nomal = np.array([pointInfo[3],pointInfo[4],pointInfo[5]])

        # point has four kinds of states: 0: non-limited, 1: margin, 2: boundary, 3: finished
        #initally, all the points are non-limited
        self.state = 0
    def calDistance(self, Point2):
        return np.linalg.norm(self.position - Point2.postion)

def build_point_and_normal_file(read_point_filename,write_point_filename, write_normal_filename):
    f = open(read_point_filename, 'rt')
    point_file = open(write_point_filename,'w')
    normal_file = open(write_normal_filename,'w')
    for line in f:
        line_split = line.split(' ')
        line_split[-1] = line_split[-1].strip('\n')
        if line_split[0] == 'v':
            point_file.write(line)
        if line_split[0] == 'vn':
            normal_file.write(line)
    f.close()
    point_file.close()
    normal_file.close()

def build_point_list(read_point_file,read_normal_file):

    Point_list = []
    point_file = open(read_point_file,'rt')
    normal_file = open(read_normal_file,'rt')
    point_info = point_file.readline()
    normal_info = normal_file.readline()
    index = 0
    def get_info(point_str,normal_str):
        point_position = point_str.split(' ')
        normal_file = normal_str.split(' ')
        return float(point_position[1]),float(point_position[2]),float(point_position[3]),[float(i) for i in normal_file[1:]]
    point_x , point_y, point_z, point_normal = get_info(point_info,normal_info)
    Point_list.append([point_x , point_y, point_z, point_normal,index])
    point_info = point_file.readline()
    normal_info = normal_file.readline()
    while point_info != '':
        index += 1
        point_x, point_y, point_z, point_normal = get_info(point_info, normal_info)
        Point_list.append([point_x, point_y, point_z, point_normal,index])
        point_info = point_file.readline()
        normal_info = normal_file.readline()
    point_file.close()
    normal_file.close()
    return Point_list


def projection_3dto2d_trangulation(inital_point,neighbourhood,min_radius = None):
    x_normal= inital_point[-2][0]
    y_normal = inital_point[-2][1]
    z_normal = inital_point[-2][2]
    projection_list = []
    for point in neighbourhood:
        t = ((inital_point[0]-point[0])*x_normal+(inital_point[1]-point[1])*y_normal +(inital_point[2]-point[2])*z_normal)\
            /(math.pow(x_normal,2)+math.pow(y_normal,2)+math.pow(z_normal,2))
        x = x_normal * t + point[0]
        y = y_normal * t + point[1]
        z = z_normal * t + point[2]
        projection_list.append([x,y,z])
    new_z_axis = np.array(inital_point[-2])
    new_x_axis = np.array([projection_list[1][i] - inital_point[:-2][i] for i in range(3)])
    new_y_axis = np.cross(new_z_axis,new_x_axis)

    normalization_x = new_x_axis / np.linalg.norm(new_x_axis)
    normalization_y = new_y_axis / np.linalg.norm(new_y_axis)
    normalization_z = new_z_axis / np.linalg.norm(new_z_axis)
    coordinate_new = np.mat([normalization_x,normalization_y,normalization_z])
    coordinate_new_I = coordinate_new.I
    result = []
    for point in projection_list:
        point2d = np.array(np.dot([point[i] - inital_point[:-2][i] for i in range(3)], coordinate_new_I))
        result.append(point2d[0][:-1])
    tri = Delaunay(result)

    face = [ [neighbourhood[i][-1] for i in j]  for j in tri.simplices]
    return face

def calDistance_point2line(vertex1,vertex2, target, dimensions = 3):
    lineVec = np.array([vertex2[i] - vertex1[i] for i in range(dimensions)] )
    vertex2Target = np.array([vertex2[i] - target[i] for i in range(dimensions)])
    S = np.linalg.norm( np.cross(lineVec, vertex2Target))
    return S / ( np.linalg.norm(lineVec))

def calDistance_point2point(vertex1, vertex2, dimensions = 3):
    vec = np.array([vertex2[i] - vertex1[i] for i in range(dimensions)] )
    return  np.linalg.norm(vec)

#if the angle less than 90^0, return true
def check_angle(vertex1,vertex2,vertex3,dimensions = 3):
    vec1 = np.array([vertex2[i] - vertex1[i] for i in range(dimensions)])
    vec2 = np.array([vertex3[i] - vertex2[i] for i in range(dimensions)])
    return (np.dot(vec1,vec2) < 0)

def maintainEdges(faceList):
    edges = []
    for face in faceList:
        if [face[0], face[1]] in edges:
            edges.remove([face[0], face[1]])
        elif [face[1], face[0]] in edges:
            edges.remove([face[1], face[0]])
        else:
            edges.append([face[0], face[1]])
        if [face[0], face[2]] in edges:
            edges.remove([face[0], face[2]])
        elif [face[2], face[0]] in edges:
            edges.remove([face[2], face[0]])
        else:
            edges.append([face[0], face[2]])
        if [face[2], face[1]] in edges:
            edges.remove([face[2], face[1]])
        elif [face[1], face[2]] in edges:
            edges.remove([face[1], face[2]])
        else:
            edges.append([face[2], face[1]])
    return edges

#O(n^2)
# we need to give a threshold Td, if the distance of the point to the edge is larger than Td, we don't add the point to candidate
def Greedy(pointList,rest, Edges,Tran, minRadius,dimensions=3):
    # it is mention in paper that a point should have four states:
    #Edges is a queue, use index to respresent edges
    trangulations = Tran[:]
    restPointList = rest[:]
    edgeQueue = queue.Queue()
    for i in Edges:
        edgeQueue.put(i)
    while len(restPointList) != 0:
        edge = edgeQueue.get(timeout=0)
        #edge(a,b) a,b is the point index in pointList
        #this list can be replaced by heap
        #distanceList = [calDistance_point2line(pointList[edge[0]][:3],pointList[edge[1]][:3],i[:3]) for i in restPointList]
        #order by the distance between points to edge got a awful performance
        # In this case, we order by the distance between points to the midpoint of the edge
        edge_point1 = pointList[edge[0]]
        edge_point2 = pointList[edge[1]]
        midpoint = [(edge_point1[i]+ edge_point2[i]) / 2 for i in range(3)]
        distanceList = [calDistance_point2point(midpoint, i[:3]) for i in restPointList]
        minDistance = min(distanceList)
        #print(minDistance)
        if minDistance > minRadius:
            continue
        #print("distance is : ", minDistance)
        min_index = distanceList.index(minDistance)
        pointAdd = restPointList[min_index]
        #In the first attempt, we just consider a point has two states: rest and done
        edgeQueue.put([pointList[edge[0]][-1],pointAdd[-1]])
        edgeQueue.put([pointList[edge[1]][-1], pointAdd[-1]])
        trangulations.append([pointList[edge[0]][-1],pointList[edge[1]][-1],pointAdd[-1]])
        restPointList.remove(pointAdd)
    return trangulations

def find_point_largestAngle(pointList, edgePoint1, edgePoint2,dimensions = 3):
    result = pointList[0]
    minial_dot = 1
    for i in pointList:
        e1 = np.array([edgePoint1[j] - i[j] for j in range(dimensions)])
        e2 = np.array([edgePoint2[j] - i[j] for j in range(dimensions)])
        normal_sum1 = 0
        normal_sum2 = 0
        for j in e1:
            normal_sum1 += j**2
        for j in e2:
            normal_sum2 += j**2
        dot_result = np.dot(e1/normal_sum1,e2/normal_sum2)
        if dot_result < minial_dot:
            minial_dot = dot_result
            result = i

    return result

#use KD tree to find the point to add
def GreedyKd(KD_tree,near_k, pointList, rest, Edges,Tran, minRadius, dimensions=3):
    restPointList = rest[:]
    trangulations = Tran[:]

    while_time = 0
    len_restList = len(restPointList)
    while len(restPointList) != 0:
        if len(restPointList) == len_restList - 1:
            while_time = 0
            len_restList = len(restPointList)
        elif len(restPointList) == len_restList:
            while_time += 1
        if while_time > len_restList:
            print("Fall in dead while.")
            break
        edge = Edges[0]
        edge_point1 = pointList[edge[0]]
        edge_point2 = pointList[edge[1]]

        midpoint = [(edge_point1[i] + edge_point2[i]) / 2 for i in range(3)]
        neighb = []
        if len(list(KD_tree.inorder())) >= near_k:
            neighb = [i[0].data for i in KD_tree.search_knn(midpoint, near_k)]
        elif len(list(KD_tree.inorder()))< near_k and len(list(KD_tree.inorder())) > 0:
            neighb = [i[0].data for i in KD_tree.search_knn(midpoint, len(list(KD_tree.inorder())))]
        else:
            break
        if edge_point1 in neighb:
            neighb.remove(edge_point1)
        if edge_point2 in neighb:
            neighb.remove(edge_point2)
        if len(neighb) == 0 or min([calDistance_point2point(midpoint, i[:3]) for i in neighb]) > minRadius:
            Edges.append(edge)
            Edges = Edges[1:]
            continue

        pointAdd = find_point_largestAngle(neighb, edge_point1, edge_point2)

        if [edge_point1, pointAdd] not in Edges and [ pointAdd, edge_point1] not in Edges:
            Edges.append([edge_point1[-1], pointAdd[-1]])
        if [edge_point2, pointAdd] not in Edges and [pointAdd, edge_point2] not in Edges:
            Edges.append([pointAdd[-1], edge_point2[-1]])
        Edges = Edges[1:]
        trangulations.append([edge_point1[-1],edge_point2[-1],pointAdd[-1]])
        restPointList.remove(pointAdd)
        KD_tree.remove(pointAdd)
    return trangulations

def GreedyTriangulation(pointList,rest, Edges,Tran,minRadius,dimensions=3):
    # it is mention in paper that a point should have four states:
    restPointList = rest[:]
    trangulations = Tran[:]
    edgePoint = {}
    for i in Edges:
        if i[0] not in edgePoint.keys():
            edgePoint[i[0]] = {}
            edgePoint[i[0]]['next'] = i[1]
        else:
            edgePoint[i[0]]['next'] = i[1]
        if i[1] not in edgePoint.keys():
            edgePoint[i[1]] = {}
            edgePoint[i[1]]['prev'] = i[0]
        else:
            edgePoint[i[1]]['prev'] = i[0]
    while_time = 0
    len_restList = len(restPointList)

    while len(restPointList) != 0:
        if len(restPointList) == len_restList-1:
            while_time = 0
            len_restList = len(restPointList)
        elif len(restPointList) == len_restList:
            while_time += 1
        if while_time > len_restList:
            print("Fall in dead while.")
            break
        edge = Edges[0]
        #edge(a,b) a,b is the point index in pointList
        #this list can be replaced by heap
        #distanceList = [calDistance_point2line(pointList[edge[0]][:3],pointList[edge[1]][:3],i[:3]) for i in restPointList]
        #order by the distance between points to edge got a awful performance
        # In this case, we order by the distance between points to the midpoint of the edge
        edge_point1 = pointList[edge[0]]
        edge_point2 = pointList[edge[1]]

        midpoint = [(edge_point1[i]+ edge_point2[i]) / 2 for i in range(3)]
        distanceList = [calDistance_point2point(midpoint, i[:3]) for i in restPointList]
        minOuterDistance = min(distanceList)

        if check_angle(edge_point1,edge_point2,pointList[edgePoint[edge[1]]['next']]):
            distance2next= calDistance_point2point(edge_point1,pointList[edgePoint[edge[1]]['next']][:3])
        else:
            distance2next = math.inf
        if check_angle(edge_point2,edge_point1,pointList[edgePoint[edge[0]]['prev']]):
            distance2prev = calDistance_point2point(edge_point2, pointList[edgePoint[edge[0]]['prev']][:3])
        else:
            distance2prev = math.inf
        #print(minOuterDistance)
        minDistance = min([minOuterDistance,distance2next,distance2prev])
        if minDistance > minRadius:
            #print('minDistance is ',minDistance)
            #print('next edge')
            Edges.append(edge)
            Edges = Edges[1:]
            continue
        #print("distance is : ", minDistance)
        Edges = Edges[1:]
        if minDistance == minOuterDistance:   # has minial distance of outer points
            min_index = distanceList.index(minDistance)
            pointAdd = restPointList[min_index]
            # print("point to add is : ", pointAdd)
            # print("points of edge is :" ,pointList[edge[0]], pointList[edge[1]])
            #In the first attempt, we just consider a point has two states: rest and done
            Edges.append([edge[0],pointAdd[-1]])
            Edges.append([pointAdd[-1],edge[1]])
            edgePoint[pointAdd[-1]] = {}
            edgePoint[pointAdd[-1]]['next'] = edge[1]
            edgePoint[pointAdd[-1]]['prev'] = edge[0]
            edgePoint[edge[1]]['prev'] = pointAdd[-1]
            edgePoint[edge[0]]['next'] = pointAdd[-1]
            trangulations.append([pointList[edge[0]][-1],pointList[edge[1]][-1],pointAdd[-1]])
            restPointList.remove(pointAdd)
        elif minDistance == distance2next:
            Edges.remove([edge[1],edgePoint[edge[1]]['next']])
            Edges.append([edge[0],edgePoint[edge[1]]['next']])
            edgePoint[edge[0]]['next'] = edgePoint[edge[1]]['next']
            edgePoint[edgePoint[edge[0]]['next'] ]['prev'] = edge[0]
            trangulations.append([pointList[edge[0]][-1],pointList[edge[1]][-1],edgePoint[edge[1]]['next']])
            del edgePoint[edge[1]]
        elif minDistance == distance2prev:
            Edges.remove([edgePoint[edge[0]]['prev'],edge[0]])
            Edges.append([edgePoint[edge[0]]['prev'],edge[1]])
            edgePoint[edge[1]]['prev'] = edgePoint[edge[0]]['prev']
            edgePoint[edgePoint[edge[1]]['prev'] ]['next'] = edge[1]
            trangulations.append([pointList[edge[0]][-1], pointList[edge[1]][-1], edgePoint[edge[0]]['prev']])
            del edgePoint[edge[0]]
    return trangulations
#One observation is that the point are ranked by the ascending order of x, y, z
# an idea is that in the process of finding the closest point to edges, we add the midpoint of edge vertices to the kd-tree
#then user the nn search to find the closed point


def write_face(faces,filename):
    f = open(filename,'w')
    for i in faces:
        f.write('f '+ str(i[0]+1) + '//' + str(i[0]+1) + ' ' +
                str(i[1]+1) + '//' + str(i[1]+1) + ' ' +  str(i[2]+1) + '//' + str(i[2]+1) + '\n' )
    f.close()
#pointList format :  x y z [nx ny nz] index

def parse_args():
    parser = argparse.ArgumentParser(description='GreedyTriangulation arguments')
    parser.add_argument('--minRadius', type=int, default='10', help='determine the minimal radius to add rest points')
    parser.add_argument('--inp_model', type=str, default="r4.obj", help='input point cloud model')
    parser.add_argument('--outp_normal', type=str, default="r4_normal.txt", help="the normal vectors of input point cloud")
    parser.add_argument('--outp_point', type=str, default="r4_point.txt", help='points of input point cloud ')

    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    filename = args.filename
    build_point_and_normal_file(filename,args.outp_point,args.outp_normal)
    #point_list = build_point_list('bone1_point.txt','bone1_normal.txt')
    point_list = build_point_list(args.outp_point,args.outp_normal)

    inital_point_index = 500
    KD_tree = kdtree.create(point_list,dimensions=3)
    k = 5
    neighbourhood = [i[0].data for i in KD_tree.search_knn(point_list[inital_point_index], k)]
    trangulations = projection_3dto2d_trangulation(point_list[inital_point_index],neighbourhood)
    inital_edges = maintainEdges(trangulations)
    insidePoint = neighbourhood[:]
    for i in inital_edges:
        for j in i:
            if point_list[j] not in insidePoint:
                continue
            insidePoint.remove(point_list[j])

    restPointList = []
    for i in range(len(point_list)):
        if point_list[i] not in insidePoint: #外部点
            restPointList.append(point_list[i])
    for i in insidePoint:
        KD_tree.remove(i)
    minRadius = args.minRadius
    trangulations2 = GreedyKd(KD_tree, k, point_list, restPointList, inital_edges, trangulations, minRadius)
    write_face(trangulations2, filename+'result_minRadius'+str(minRadius)+ '_GreedyKD.txt')

if __name__ == '__main__':
    main()