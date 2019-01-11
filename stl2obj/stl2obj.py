# 文件起始的80个字节是文件头，用于存贮文件名；
# 紧接着用 4 个字节的整数来描述模型的三角面片个数，
# 后面逐个给出每个三角面片的几何信息。每个三角面片占用固定的50个字节，依次是:
# 3个4字节浮点数(角面片的法矢量)
# 3个4字节浮点数(1个顶点的坐标)
# 3个4字节浮点数(2个顶点的坐标)
# 3个4字节浮点数(3个顶点的坐标)个
# 三角面片的最后2个字节用来描述三角面片的属性信息。

#发现已有的法向量都是0
import struct

def isPointInDict(x,y,z,pointsDic):
    return (x,y,z) in pointsDic.keys() or (x,z,y) in pointsDic.keys() or \
           (y,x,z) in pointsDic.keys() or (y,z,x) in pointsDic.keys() or \
           (z,x,y) in pointsDic.keys() or (z,y,x) in pointsDic.keys()
def stl2obj(read_stl_filename, write_obj_filename):
    file_stl = open(read_stl_filename,'rb')
    file_obj = open(write_obj_filename,'w')
    i=0
    #since in stl, a point may appear multi-times
    pointDic = {}
    faceList = []
    index = 1
    while i < 80:  # 前80个字节是文件头，不存
        file_stl.read(1)  # 读一个字节
        i = i + 1
    c = file_stl.read(4)
    faces_num = struct.unpack('<i', c)[0]
    print("The total number of the faces is ", faces_num)
    while 1:
        c = file_stl.read(4)
        if not c:
            break
        normal_x = struct.unpack('<f', c)[0]
        normal_y = struct.unpack('<f', file_stl.read(4))[0]
        normal_z = struct.unpack('<f', file_stl.read(4))[0]

        point1_x = struct.unpack('<f', file_stl.read(4))[0]
        point1_y = struct.unpack('<f', file_stl.read(4))[0]
        point1_z = struct.unpack('<f', file_stl.read(4))[0]

        point2_x = struct.unpack('<f', file_stl.read(4))[0]
        point2_y = struct.unpack('<f', file_stl.read(4))[0]
        point2_z = struct.unpack('<f', file_stl.read(4))[0]

        point3_x = struct.unpack('<f', file_stl.read(4))[0]
        point3_y = struct.unpack('<f', file_stl.read(4))[0]
        point3_z = struct.unpack('<f', file_stl.read(4))[0]

        others = file_stl.read(2)

        #c 是一个字节， 两位十六进制数
        #三个 四字节浮点数好解决，关键是 一个面片的法向量怎么搞？这是一个面片的法向量，而obj中的法向量呢？ obj 中的法向量是每个点都有的吗？
        #事实证明是每个点都有的， obj格式的文件中，有多少个顶点，就有对应顶点的法向量。
        #想法之一是每个小面元的三个顶点的法向量就等于这个面元的法向量

        if not isPointInDict(point1_x,point1_y,point1_z,pointDic):
            pointDic[(point1_x,point1_y,point1_z)] = (normal_x,normal_y,normal_z, index)
            index += 1
        if not isPointInDict(point2_x,point2_y,point2_z,pointDic):
            pointDic[(point2_x,point2_y,point2_z)] = (normal_x,normal_y,normal_z, index)
            index += 1
        if not isPointInDict(point3_x,point3_y,point3_z,pointDic):
            pointDic[(point3_x,point3_y,point3_z)] = (normal_x,normal_y,normal_z, index)
            index += 1
        faceList.append([pointDic[(point1_x,point1_y,point1_z)][-1],pointDic[(point2_x,point2_y,point2_z)][-1],pointDic[(point3_x,point3_y,point3_z)][-1]])
    for i in pointDic.keys():
        file_obj.write('v '+ str(i[0]) + ' ' + str(i[1]) + ' ' + str(i[2]) + '\n')
    for i in pointDic.keys():
        file_obj.write('vn ' + str(pointDic[i][0]) + ' ' + str(pointDic[i][1]) + ' ' + str(pointDic[i][2]) + '\n')
    file_obj.write("\ng grp1\nusemtl mtl1\n")
    for i in faceList:
        file_obj.write('f ' + str(i[0]) + '//' + str(i[0]) + ' ' +
                       str(i[1]) + '//' + str(i[1]) + ' ' +
                       str(i[2]) + '//' + str(i[2]) + '\n')

    file_obj.close()
    file_stl.close()

def main():
    read_stl_filename = 'U.stl'
    write_obj_filename = 'U.obj'
    stl2obj(read_stl_filename,write_obj_filename)

if __name__ == '__main__':
    main()
# dic_try = {}
# dic_try[((1,2,3),(1,2,3),(1,2,3))] = 1