#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

float right(float x_offset, int k, float top_right_lon, vector<vector<float> > &point){
    float x_temp = point.back()[0];
    float y_temp = point.back()[1];
    while(true) {
        x_temp += x_offset / float(k);
        if (x_temp < top_right_lon) {
            point.push_back({x_temp,y_temp});
        }
        else
            break;
    }
    return (x_temp-x_offset / float(k));  // 返回最边缘的点的lon值
}

float left(float x_offset, int k, float bottom_left_lon, vector<vector<float> > &point){
    float x_temp = point.back()[0];
    float y_temp = point.back()[1];
    while(true) {
        x_temp -= x_offset / float(k);
        if (x_temp > bottom_left_lon) {
            point.push_back({x_temp,y_temp});
        }
        else
            break;
    }
    return (x_temp+x_offset / float(k));  // 返回最边缘的点的lon值
}

/// 按顺序生成ROV目标点
/// \param center_point_lon 输入：正方形区域中心点经度
/// \param center_point_lat 输入：正方形区域中心点纬度
/// \param half_length 输入：正方形区域一半长
/// \param k 输入：每一行生成圆的数量
/// \param point 输出：按顺序排列的点的集合
void Generate_target_point(float center_point_lon, float center_point_lat, float half_length, int k, vector<vector<float> > &point){

    float bottom_left_lon = center_point_lon-half_length;
    float bottom_left_lat = center_point_lat-half_length;
    float top_right_lon = bottom_left_lon+2*half_length;
    float top_right_lat = bottom_left_lat+2*half_length;

    // M = {{26.0,111.0}, {26.003,111.0}, {26.003,111.003}, {26.0,111.003}};
    // M = {{0,0}, {10,0}, {10,10}, {0,10}};

    float x_offset = top_right_lon-bottom_left_lon;
    float y_offset = top_right_lat-bottom_left_lat;

    float start_point_x = bottom_left_lon+(x_offset/float(k))/2;
    float start_point_y = bottom_left_lat+(y_offset/float(k))/2;
    point.push_back({start_point_x, start_point_y});  // 插入第一个点

    while (true) {
        float x1 = right(x_offset, k, top_right_lon, point);  // 向右
        if (point.back()[1] + x_offset / float(k) >= top_right_lat)  // 判断lat是否超过边界
            break;
        point.push_back({x1, point.back()[1] + x_offset / float(k)});

        float x2 = left(x_offset, k, bottom_left_lon, point);  // 向左
        if (point.back()[1] + x_offset / float(k) >= top_right_lat)  // 判断lat是否超过边界
            break;
        point.push_back({x2, point.back()[1] + x_offset / float(k)});
    }
}

int main(){
    vector<vector<float> > point;

    //center_point_lon 输入：正方形区域中心点经度
    //center_point_lat 输入：正方形区域中心点纬度
    //half_length 输入：正方形区域一半长
    //k 输入：每一行生成圆的数量
    //point 输出：按顺序排列的点的集合
    Generate_target_point(26.0015, 111.0015, 0.0010, 3, point);
    ofstream file;
    file.open("/home/zyf/data.csv",std::ios::out | std::ios::trunc);
    for (const auto& i :point){
        for (auto j :i){
            cout.precision(10);
            file.precision(10);
            cout<<j<<"\t";
            file<<j<<",";
        }
        cout<<endl;
        file<<endl;
    }
    return 0;
}



