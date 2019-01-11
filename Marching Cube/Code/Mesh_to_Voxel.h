#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory.h>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
using namespace std;

int unit_ijk[8][3] = {
    {0, 0, 0},
    {1, 0, 0},
    {0, 0, 1},
    {1, 0, 1},
    {0, 1, 0},
    {1, 1, 0},
    {0, 1, 1},
    {1, 1, 1}};

int unit_face[12][3] = {
    {7, 1, 3},
    {5, 1, 7},
    {4, 2, 8},
    {8, 2, 6},
    {4, 7, 3},
    {8, 7, 4},
    {1, 5, 2},
    {2, 5, 6},
    {1, 4, 3},
    {2, 4, 1},
    {7, 8, 5},
    {5, 8, 6}};

int unit_normal_vector[6][3] = {
    {1, 1, 1},
    {2, 2, 2},
    {3, 3, 3},
    {4, 4, 4},
    {5, 5, 5},
    {6, 6, 6}};

struct point
{
    float x;
    float y;
    float z;
    point(float xx = 0.0, float yy = 0.0, float zz = 0.0) : x(xx), y(yy), z(zz) {}
    point(const point &v) : x(v.x), y(v.y), z(v.z) {}
};

point operator+(const point &v1, const point &v2)
{
    return point(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

struct normal_vector
{
    float nx;
    float ny;
    float nz;
};

struct face
{
    int v[3];
    int *nv;
};

struct OBJ
{
    vector<point> v;
    vector<normal_vector> nv;
    vector<face> f;
};

bool compare_x(const point &a, const point &b)
{
    return a.x < b.x;
}

bool compare_y(const point &a, const point &b)
{
    return a.y < b.y;
}

bool compare_z(const point &a, const point &b)
{
    return a.z < b.z;
}

float unit_nv[6][3] = {
    {-1.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {0, 0.0, 1.0},
    {0.0, 0.0, -1.0},
    {0.0, -1.0, 0.0},
    {0, 1.0, 0.0}};


float upper_bound_x, lower_bound_x, upper_bound_y, lower_bound_y, upper_bound_z, lower_bound_z;

float lower_bound_box_x, lower_bound_box_y, lower_bound_box_z;

float unit_length;

int x_length, y_length, z_length;

string output_filename, input_filename;

deque<deque<deque<bool>>> unit;

OBJ obj_mesh, obj_voxel;