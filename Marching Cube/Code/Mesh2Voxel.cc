#include "Mesh_to_Voxel.h"

void read_obj_file(string file)
{
    cout << "read file " + file << endl;
    int filename_length = file.size();
    int index_filename = filename_length;
    int index_i = filename_length;
    while (index_filename != 0 && file[index_filename - 1] != '/' && file[index_filename - 1] != '\\')
    {
        --index_filename;
    }
    while (file[--index_i] != '.');
    for (int i = index_filename; i < index_i; ++i) {
        output_filename += file[i];
    }
    ifstream ifs(file);
    if (!ifs)
    {
        cout << "Open file Error!";
        return;
    }
    string line;
    string head;
    face *f;
    point *v;
    normal_vector *nv;
    while (getline(ifs, line))
    {
        if (line.length() < 2)
            continue;
        if (line[0] == 'v')
        {
            /* normal vector */
            if (line[1] == 'n')
            {
                istringstream input(line);
                nv = new normal_vector();
                input >> head >> nv->nx >> nv->ny >> nv->nz;
                obj_mesh.nv.push_back(*nv);
            }
            /* point vector */
            else
            {
                istringstream input(line);
                v = new point();
                input >> head >> v->x >> v->y >> v->z;

                obj_mesh.v.push_back(*v);
            }
        }
        else if (line[0] == 'f')
        {
            replace(line.begin(), line.end(), '/', ' ');
            istringstream input(line);
            f = new face();
            f->nv = new int[3];
            input >> head >> f->v[0] >> f->nv[0] >> f->v[1] >> f->nv[1] >> f->v[2] >> f->nv[2];
            f->v[0] -= 1;
            f->nv[0] -= 1;
            f->v[1] -= 1;
            f->nv[1] -= 1;
            f->v[2] -= 1;
            f->nv[2] -= 1;
            obj_mesh.f.push_back(*f);
        }
    }
    ifs.close();
    cout << "read process finished" << endl;
}

void grid_init(int x_count)
{
    //int total_vertex_num = obj_mesh.v.size();
    upper_bound_x = max_element(obj_mesh.v.begin(), obj_mesh.v.end(), compare_x)->x;
    upper_bound_y = max_element(obj_mesh.v.begin(), obj_mesh.v.end(), compare_y)->y;
    upper_bound_z = max_element(obj_mesh.v.begin(), obj_mesh.v.end(), compare_z)->z;

    lower_bound_x = min_element(obj_mesh.v.begin(), obj_mesh.v.end(), compare_x)->x;
    lower_bound_y = min_element(obj_mesh.v.begin(), obj_mesh.v.end(), compare_y)->y;
    lower_bound_z = min_element(obj_mesh.v.begin(), obj_mesh.v.end(), compare_z)->z;

    //point point_sum = accumulate(obj_mesh.v.begin(), obj_mesh.v.end(), point());
    //printf("average x : %f\naverage y : %f\naverage z: %f\n", point_sum.x / total_vertex_num, point_sum.y / total_vertex_num, point_sum.z / total_vertex_num);
    //printf("max x : %f\nmax y : %f\nmax z: %f\n", upper_bound_x, upper_bound_y, upper_bound_z);
    //printf("min x : %f\nmin y : %f\nmin z: %f\n", lower_bound_x, lower_bound_y, lower_bound_z);

    x_length = x_count - 1;
    unit_length = (upper_bound_x - lower_bound_x) / x_length;
    y_length = (upper_bound_y - lower_bound_y) / (upper_bound_x - lower_bound_x) * x_length;
    z_length = (upper_bound_z - lower_bound_z) / (upper_bound_x - lower_bound_x) * x_length;
    ++x_length;
    ++y_length;
    ++z_length;
    char tmp_filename[50];
    sprintf(tmp_filename, "%s_%dx%dx%d.obj", output_filename.c_str(), x_length, y_length, z_length);
    output_filename = tmp_filename;

    lower_bound_box_x = lower_bound_x - 0.5 * unit_length;
    lower_bound_box_y = lower_bound_y - 0.5 * unit_length;
    lower_bound_box_z = lower_bound_z - 0.5 * unit_length;

    cout << "unit length = " << unit_length << endl;
    cout << "x direction has " << x_length << " cubes" << endl;
    cout << "y direction has " << y_length << " cubes" << endl;
    cout << "z direction has " << z_length << " cubes" << endl;
    cout << "upper_bound_x = " << upper_bound_x << endl;
    cout << "lower_bound_x = " << lower_bound_x << endl;
    unit.resize(x_length);
    for (int i = 0; i < x_length; ++i)
    {
        unit[i].resize(y_length);
    }
    for (int i = 0; i < x_length; ++i)
    {
        for (int j = 0; j < y_length; ++j)
        {
            unit[i][j].resize(z_length, false);
        }
    }
    cout << "grid init" << endl;
}

void surface_voxelize()
{
    int face_number = obj_mesh.f.size();

    /**********************算法待优化***********************************************/
    for (int i = 0; i < face_number; ++i)
    {
        int index_x = ((obj_mesh.v[obj_mesh.f[i].v[0]].x + obj_mesh.v[obj_mesh.f[i].v[1]].x + obj_mesh.v[obj_mesh.f[i].v[2]].x) / 3 - lower_bound_x) / unit_length;
        int index_y = ((obj_mesh.v[obj_mesh.f[i].v[0]].y + obj_mesh.v[obj_mesh.f[i].v[1]].y + obj_mesh.v[obj_mesh.f[i].v[2]].y) / 3 - lower_bound_y) / unit_length;
        int index_z = ((obj_mesh.v[obj_mesh.f[i].v[0]].z + obj_mesh.v[obj_mesh.f[i].v[1]].z + obj_mesh.v[obj_mesh.f[i].v[2]].z) / 3 - lower_bound_z) / unit_length;
        unit[index_x][index_y][index_z] = true;
    }
    /******************************************************************************/
    cout << "surface voxelize" << endl;
}



void draw_cube(int index_i, int index_j, int index_k)
{
    face *f;
    int vertex_num = obj_voxel.v.size();
    for (int i = 0; i < 8; ++i)
    {
        point *v = new point();
        v->x = lower_bound_x + (index_i + unit_ijk[i][0]) * unit_length;
        v->y = lower_bound_y + (index_j + unit_ijk[i][1]) * unit_length;
        v->z = lower_bound_z + (index_k + unit_ijk[i][2]) * unit_length;
        obj_voxel.v.push_back(*v);
    }
    for (int i = 0; i < 12; ++i)
    {
        f = new face();
        f->nv = unit_normal_vector[i >> 1];
        for (int j = 0; j < 3; ++j)
        {
            f->v[j] = vertex_num + unit_face[i][j];
        }
        obj_voxel.f.push_back(*f);
    }
}

void calculate_elements()
{
    cout << "calculate cubes" << endl;
    int count = 0;
    for (int i = 0; i < x_length; ++i)
    {
        for (int j = 0; j < y_length; ++j)
        {
            for (int k = 0; k < z_length; ++k)
            {
                if (unit[i][j][k])
                {
                    draw_cube(i, j, k);
                    ++count;
                    if (count % 100 == 0)
                    {
                        printf("%7d cubes totally\r", count);
                    }
                }
            }
        }
    }
    unit.clear();
    printf("%7d cubes totally\n", count);
    printf("calculate cubes finished\n");
}

void output_to_obj_file(string file)
{
    printf("output to %s\r", output_filename.c_str());
    ofstream ofs(file);
    if (!ofs)
    {
        cout << "Open file Error!";
        return;
    }
    ofs << "# voxel file transformed from triangle mesh file, implemented by Shengfa Zhang, from Group AG_2\n";

    ofs << "\n# vertices\n";
    int vertex_num = obj_voxel.v.size();
    for (int i = 0; i < vertex_num; ++i)
    {
        ofs << "v " << obj_voxel.v[i].x << ' ' << obj_voxel.v[i].y << ' ' << obj_voxel.v[i].z << endl;
    }

    ofs << "\n# normals\n";
    for (int i = 0; i < 6; ++i)
    {
        ofs << "vn " << unit_nv[i][0] << ' ' << unit_nv[i][1] << ' ' << unit_nv[i][2] << endl;
    }
    int face_num = obj_voxel.f.size();
    ofs << "\n# faces\n";
    for (int i = 0; i < face_num; ++i)
    {
        ofs << "f " << obj_voxel.f[i].v[0] << "//" << obj_voxel.f[i].nv[0] << ' '
            << obj_voxel.f[i].v[1] << "//" << obj_voxel.f[i].nv[1] << ' '
            << obj_voxel.f[i].v[2] << "//" << obj_voxel.f[i].nv[2] << endl;
    }
    ofs.close();
    printf("output to %s successfully\n", output_filename.c_str());
}

int main()
{
    //read_obj_file("G:/2018_FallTerm/Computer Graphics/AG/AG/OBJ/bone1.obj");
    read_obj_file("skin2.obj");
    grid_init(50);
    surface_voxelize();
    calculate_elements();
    //output_to_obj_file("bone1_voxel.obj");
    output_to_obj_file(output_filename);
    return 0;
}