def delete_f(read_file, write_file):
    obj_file = open(read_file,'rt')
    write_obj_file = open(write_file,'w')
    for line in obj_file:
        if line[0] == 'f':
            continue
        write_obj_file.write(line)
    obj_file.close()
    write_obj_file.close()

def delete_n_curve(read_file,write_file):
    obj_file = open(read_file, 'rt')
    write_obj_file = open(write_file, 'w')
    flag = False
    for line in obj_file:
        if flag:
            line_strip = line.strip('\n').split(' ')
            if line_strip[0] == '3':
                flag = False
                write_obj_file.write(line)
                continue
            new_string = line_strip[0]+' '+line_strip[1] + line_strip[2] +'\n'
            write_obj_file.write(new_string)
        else:
            write_obj_file.write(line)
        if line == 'end_header\n':
            flag = True


def count_point_number_and_normal(read_file):
    obj_file = open(read_file, 'rt')
    vertex_count = 0
    normal_count = 0
    for line in obj_file:
        line_strip = line.strip('\n').split(' ')
        if line_strip[0] == 'v':
            vertex_count += 1
        if line_strip[0] == 'vn':
            normal_count += 1
    print('vertex number: ', vertex_count)
    print('normal number: ', normal_count)

def main():
    read_file = 'bone1.obj'
    write_file = 'bone1_p.obj'
    delete_f(read_file,write_file)

    count_point_number_and_normal('bone1.obj')

if __name__ == '__main__':
    main()
