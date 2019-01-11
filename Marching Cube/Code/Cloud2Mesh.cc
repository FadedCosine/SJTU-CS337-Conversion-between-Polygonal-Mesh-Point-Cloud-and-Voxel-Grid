#include "Cloud_to_Mesh.h"

void read_obj_file(string file)
{
    ifstream ifs(file);
    if (!ifs)
    {
        cout << "Open file Error!";
        return;
    }
    printf("read file %s\n", file.c_str());
    string line;
    string head;
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
                input >> head >> tmp_nv.nx >> tmp_nv.ny >> tmp_nv.nz;
                obj_voxel.nv.push_back(tmp_nv);
            }
            /* point vector */
            else
            {
                istringstream input(line);
                input >> head >> tmp_v.x >> tmp_v.y >> tmp_v.z;

                obj_voxel.v.push_back(tmp_v);
            }
        }
    }
    ifs.close();
    printf("read process finished!\n");
}

void grid_init(int x_count)
{
    upper_bound_x = max_element(obj_voxel.v.begin(), obj_voxel.v.end(), compare_x)->x;
    upper_bound_y = max_element(obj_voxel.v.begin(), obj_voxel.v.end(), compare_y)->y;
    upper_bound_z = max_element(obj_voxel.v.begin(), obj_voxel.v.end(), compare_z)->z;

    lower_bound_x = min_element(obj_voxel.v.begin(), obj_voxel.v.end(), compare_x)->x;
    lower_bound_y = min_element(obj_voxel.v.begin(), obj_voxel.v.end(), compare_y)->y;
    lower_bound_z = min_element(obj_voxel.v.begin(), obj_voxel.v.end(), compare_z)->z;

    x_length = x_count - 1;
    unit_length = (upper_bound_x - lower_bound_x) / x_length;
    y_length = (upper_bound_y - lower_bound_y + 0.5 * unit_length) / unit_length;
    z_length = (upper_bound_z - lower_bound_z + 0.5 * unit_length) / unit_length;
    x_length += 1;
    y_length += 1;
    z_length += 1;

    lower_bound_box_x = lower_bound_x - 0.5 * unit_length;
    lower_bound_box_y = lower_bound_y - 0.5 * unit_length;
    lower_bound_box_z = lower_bound_z - 0.5 * unit_length;

    cout << "unit length = " << unit_length << endl;
    cout << "x direction has " << x_length << " cubes" << endl;
    cout << "y direction has " << y_length << " cubes" << endl;
    cout << "z direction has " << z_length << " cubes" << endl;

    unit_value.resize(x_length + 3);
    for (int i = 0; i < x_length + 3; ++i)
    {
        unit_value[i].resize(y_length + 3);
    }
    for (int i = 0; i < x_length + 3; ++i)
    {
        for (int j = 0; j < y_length + 3; ++j)
        {
            unit_value[i][j].resize(z_length + 3, 0);
        }
    }

    cout << "grid init finished" << endl;
}

void calculate_unit()
{
    int vertex_number = obj_voxel.v.size();
    for (int i = 0; i < vertex_number; ++i)
    {
        int index_x = (obj_voxel.v[i].x - lower_bound_box_x) / unit_length;
        int index_y = (obj_voxel.v[i].y - lower_bound_box_y) / unit_length;
        int index_z = (obj_voxel.v[i].z - lower_bound_box_z) / unit_length;
        for (int l = 0; l < 8; ++l)
        {
            unit_value[index_x + unit_grid[l][0] + 1][index_y + unit_grid[l][1] + 1][index_z + unit_grid[l][2] + 1] += 1;
        }
    }
}

point calculate_vertex(double iso, point p1, point p2, double val1, double val2)
{
    double lamda;
    point p;
    if (iso - val1 < 0.00001 && val1 - iso < 0.00001)
        return p1;
    if (iso - val2 < 0.00001 && val2 - iso < 0.00001)
        return p2;
    if (val1 - val2 < 0.00001 && val2 - val1 < 0.00001)
        return p1;
    lamda = (iso - val1) / (val2 - val1);
    p.x = p1.x + lamda * (p2.x - p1.x);
    p.y = p1.y + lamda * (p2.y - p1.y);
    p.z = p1.z + lamda * (p2.z - p1.z);

    return p;
}

void calculate_normal(const point &p1, const point &p2, const point &p3)
{
    double x = (p2.y - p1.y) * (p3.z - p2.z) - (p2.z - p1.z) * (p3.y - p2.y);
    double y = (p2.z - p1.z) * (p3.x - p2.x) - (p2.x - p1.x) * (p3.z - p2.z);
    double z = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
    double length = sqrt(x * x + y * y + z * z);
    tmp_nv.nx = x / length;
    tmp_nv.ny = y / length;
    tmp_nv.nz = z / length;
}

void PolygoniseCube(grid_cell g, double iso)
{
    int cubeindex = 0;
    point triangle[3];
    point vertices[12];
    {
        if (g.val[0] < iso)
            cubeindex |= 1;
        if (g.val[1] < iso)
            cubeindex |= 2;
        if (g.val[2] < iso)
            cubeindex |= 4;
        if (g.val[3] < iso)
            cubeindex |= 8;
        if (g.val[4] < iso)
            cubeindex |= 16;
        if (g.val[5] < iso)
            cubeindex |= 32;
        if (g.val[6] < iso)
            cubeindex |= 64;
        if (g.val[7] < iso)
            cubeindex |= 128;
    }
    if (edgeTable[cubeindex] == 0)
        return;
    {
        if (edgeTable[cubeindex] & 1)
        {
            vertices[0] = calculate_vertex(iso, g.p[0], g.p[1], g.val[0], g.val[1]);
        }
        if (edgeTable[cubeindex] & 2)
        {
            vertices[1] = calculate_vertex(iso, g.p[1], g.p[2], g.val[1], g.val[2]);
        }
        if (edgeTable[cubeindex] & 4)
        {
            vertices[2] = calculate_vertex(iso, g.p[2], g.p[3], g.val[2], g.val[3]);
        }
        if (edgeTable[cubeindex] & 8)
        {
            vertices[3] = calculate_vertex(iso, g.p[3], g.p[0], g.val[3], g.val[0]);
        }
        if (edgeTable[cubeindex] & 16)
        {
            vertices[4] = calculate_vertex(iso, g.p[4], g.p[5], g.val[4], g.val[5]);
        }
        if (edgeTable[cubeindex] & 32)
        {
            vertices[5] = calculate_vertex(iso, g.p[5], g.p[6], g.val[5], g.val[6]);
        }
        if (edgeTable[cubeindex] & 64)
        {
            vertices[6] = calculate_vertex(iso, g.p[6], g.p[7], g.val[6], g.val[7]);
        }
        if (edgeTable[cubeindex] & 128)
        {
            vertices[7] = calculate_vertex(iso, g.p[7], g.p[4], g.val[7], g.val[4]);
        }
        if (edgeTable[cubeindex] & 256)
        {
            vertices[8] = calculate_vertex(iso, g.p[0], g.p[4], g.val[0], g.val[4]);
        }
        if (edgeTable[cubeindex] & 512)
        {
            vertices[9] = calculate_vertex(iso, g.p[1], g.p[5], g.val[1], g.val[5]);
        }
        if (edgeTable[cubeindex] & 1024)
        {
            vertices[10] = calculate_vertex(iso, g.p[2], g.p[6], g.val[2], g.val[6]);
        }
        if (edgeTable[cubeindex] & 2048)
        {
            vertices[11] = calculate_vertex(iso, g.p[3], g.p[7], g.val[3], g.val[7]);
        }
    }
    for (int i = 0; triTable[cubeindex][i] != -1; i += 3)
    {
        for (int j = 0; j < 3; ++j)
        {
            triangle[j] = vertices[triTable[cubeindex][i + j]];
            triangle[j].x = lower_bound_x + unit_length * triangle[j].x;
            triangle[j].y = lower_bound_y + unit_length * triangle[j].y;
            triangle[j].z = lower_bound_z + unit_length * triangle[j].z;

            if (v_index.count(triangle[j]) == 0)
            {
                ++v_count;
                v_index[triangle[j]] = v_count;
                obj_mesh.v.push_back(triangle[j]);
            }
        }
        calculate_normal(triangle[0], triangle[1], triangle[2]);

        ++tri_num;
        for (int j = 0; j < 3; ++j)
        {
            normal_set[triangle[j]].push_back(tmp_nv);
            face_index[tri_num][j] = triangle[j];
        }
    }
}

void calculate_triangle_mesh(double iso)
{
    cout << "calculate triangle mesh" << endl;
    grid_cell grid;
    for (int i = 0; i < x_length + 2; ++i)
    {
        for (int j = 0; j < y_length + 2; ++j)
        {
            for (int k = 0; k < z_length + 2; ++k)
            {
                for (int l = 0; l < 8; ++l)
                {
                    grid.p[l].x = i + unit_grid[l][0];
                    grid.p[l].y = j + unit_grid[l][1];
                    grid.p[l].z = k + unit_grid[l][2];
                    grid.val[l] = unit_value[grid.p[l].x][grid.p[l].y][grid.p[l].z];
                }
                PolygoniseCube(grid, iso);
            }
        }
    }
    unit_value.clear();
    cout << "calculate triangle mesh finished" << endl;
}

void calculate_normal_value()
{
    cout << "calculate normal value" << endl;
    normal_vector zero;
    int obj_mesh_vertex_num = obj_mesh.v.size();
    for (int i = 0; i < obj_mesh_vertex_num; ++i)
    {
        if (normal_value.find(obj_mesh.v[i]) == normal_value.end())
        {
            normal_value[obj_mesh.v[i]] =
                accumulate(normal_set[obj_mesh.v[i]].begin(), normal_set[obj_mesh.v[i]].end(), zero) / normal_set[obj_mesh.v[i]].size();
            /*cout << "normal_value = (" << normal_value[obj_mesh.v[i]].nx << ", "
                 << normal_value[obj_mesh.v[i]].ny << ", "
                 << normal_value[obj_mesh.v[i]].nz << ")" << endl;*/
        }
        if (nv_index.count(normal_value[obj_mesh.v[i]]) == 0)
        {
            ++nv_count;
            nv_index[normal_value[obj_mesh.v[i]]] = nv_count;
            obj_mesh.nv.push_back(normal_value[obj_mesh.v[i]]);
        }
    }
    cout << "calculate normal value finished" << endl;
}

void calculate_face_index()
{
    cout << "calculate face index " << endl;
    for (int i = 1; i <= tri_num; ++i)
    {
        tmp_f.v[0] = v_index[face_index[i][0]];
        tmp_f.v[1] = v_index[face_index[i][1]];
        tmp_f.v[2] = v_index[face_index[i][2]];
        tmp_f.nv[0] = nv_index[normal_value[face_index[i][0]]];
        tmp_f.nv[1] = nv_index[normal_value[face_index[i][1]]];
        tmp_f.nv[2] = nv_index[normal_value[face_index[i][2]]];
        obj_mesh.f.push_back(tmp_f);
    }

    v_index.clear();
    nv_index.clear();
    face_index.clear();
    normal_value.clear();

    cout << "calculate face index finished" << endl;
}

void output_to_obj_file(string file)
{
    printf("output to %s\r", file.c_str());
    ofstream ofs(file);
    if (!ofs)
    {
        cout << "Open file Error!";
        return;
    }
    ofs << "# voxel file transformed from triangle mesh file, implemented by Group AG_2\n";

    ofs << "\n# vertices\n";
    int vertex_num = obj_mesh.v.size();
    int normal_vertex_num = obj_mesh.nv.size();
    for (int i = 0; i < vertex_num; ++i)
    {
        ofs << "v " << obj_mesh.v[i].x << ' ' << obj_mesh.v[i].y << ' ' << obj_mesh.v[i].z << endl;
    }

    ofs << "\n# normals\n";
    for (int i = 0; i < normal_vertex_num; ++i)
    {
        ofs << "vn " << obj_mesh.nv[i].nx << ' ' << obj_mesh.nv[i].ny << ' ' << obj_mesh.nv[i].nz << endl;
    }
    int face_num = obj_mesh.f.size();
    ofs << "\n# faces\n";
    for (int i = 0; i < face_num; ++i)
    {
        ofs << "f " << obj_mesh.f[i].v[0] << "//" << obj_mesh.f[i].nv[0] << ' '
            << obj_mesh.f[i].v[1] << "//" << obj_mesh.f[i].nv[1] << ' '
            << obj_mesh.f[i].v[2] << "//" << obj_mesh.f[i].nv[2] << endl;
    }
    ofs.close();
    printf("output to %s successfully\n", file.c_str());
}

int main()
{
    string filename = "U_cloud.obj";
    double iso = 0.5;
    int x_count = 50;
    char output_file[50];
    read_obj_file(filename);
    grid_init(x_count);
    calculate_unit();
    calculate_triangle_mesh(iso);
    calculate_normal_value();
    calculate_face_index();
    sprintf_s(output_file, "C2M_iso%.1lf_x%d_%s", iso, x_count, filename.c_str());
    output_filename = string(output_file);
    cout << output_filename << endl;
    output_to_obj_file(output_filename);
    return 0;
}