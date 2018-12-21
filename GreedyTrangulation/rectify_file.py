

def rectify_index(read_filename, write_filename):
    read_file = open(read_filename,'rt')
    write_file = open(write_filename,'w')
    for line in read_file:
        line_split = line.strip('\n').split(' ')
        write_file.write('f '+ str(int(line_split[1].split('//')[0])+1)+'//'+str(int(line_split[1].split('//')[0])+1)+' ' +
            str(int(line_split[2].split('//')[0]) + 1) + '//' + str(int(line_split[2].split('//')[0]) + 1) + ' ' +
            str(int(line_split[3].split('//')[0]) + 1) + '//' + str(int(line_split[3].split('//')[0]) + 1) + '\n')

    read_file.close()
    write_file.close()

def main():
    rectify_index('result_test.txt','result_test_rectify.txt')

if __name__ == '__main__':
    main()