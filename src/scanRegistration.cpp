// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//fast-lio2
#include "fast-lio2/preprocess.h"
#include <mutex>
#include <signal.h>
#include <condition_variable>
using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

vector<size_t> plane_feature_size;
vector<size_t> edge_feature_size;

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; 

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

/**********   from   fast-lio2     ***********/
/**********   from   fast-lio2     ***********/
/**********   from   fast-lio2     ***********/
////int lidar_type, point_filter_num, N_SCANS, ;
//typedef pcl::PointXYZINormal PointType_XYZIN;
//typedef pcl::PointCloud<PointType_XYZIN> PointCloudXYZI;
//int SCAN_RATE = 10;
//bool feature_enabled = true;
//bool given_offset_time = false;
//
//PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
//
//
//enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
//enum Surround{Prev, Next};
//enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};
//
////用于记录每个点的距离、角度、特征种类等属性
//struct orgtype
//{
//    double range; //平面距离
//    double dista; //与后一个点的距离
//    double angle[2];
//    double intersect;
//    E_jump edj[2];
//    Feature ftype;
//    orgtype()
//    {
//        range = 0;
//        edj[Prev] = Nr_nor;
//        edj[Next] = Nr_nor;
//        ftype = Nor;
//        intersect = 2;
//    }
//};
//std::vector<orgtype> typess[128]; //maximum 128 line lidar
//
//namespace velodyne_ros {
//    struct EIGEN_ALIGN16 Point {
//        PCL_ADD_POINT4D;
//        float intensity;
//        float time;
//        uint16_t ring;
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    };
//}  // namespace velodyne_ros
//
//double blind = 4;
//int group_size = 8; //计算特征时需要的最少点数
//
//void give_feature(pcl::PointCloud<PointType_XYZIN> &pl, std::vector<orgtype> &types)
//{
//    int plsize = pl.size();
//    int plsize2; //用于估计特征的点数
//    if(plsize == 0)
//    {
//        printf("something wrong\n");
//        return;
//    }
//    uint head = 0;
//
//    while(types[head].range < blind)
//    {
//        head++;
//    }
//
//    // Surf
//    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;
//
//    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
//    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());
//
//    uint i_nex = 0, i2;
//    uint last_i = 0; uint last_i_nex = 0;
//    int last_state = 0;
//    int plane_type;
//
//    for(uint i=head; i<plsize2; i++)
//    {
//        if(types[i].range < blind)
//        {
//            continue;
//        }
//
//        i2 = i;
//
//        plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
//
//        if(plane_type == 1)
//        {
//            for(uint j=i; j<=i_nex; j++)
//            {
//                if(j!=i && j!=i_nex)
//                {
//                    types[j].ftype = Real_Plane;
//                }
//                else
//                {
//                    types[j].ftype = Poss_Plane;
//                }
//            }
//
//            // if(last_state==1 && fabs(last_direct.sum())>0.5)
//            if(last_state==1 && last_direct.norm()>0.1)
//            {
//                double mod = last_direct.transpose() * curr_direct;
//                if(mod>-0.707 && mod<0.707)
//                {
//                    types[i].ftype = Edge_Plane;
//                }
//                else
//                {
//                    types[i].ftype = Real_Plane;
//                }
//            }
//
//            i = i_nex - 1;
//            last_state = 1;
//        }
//        else // if(plane_type == 2)
//        {
//            i = i_nex;
//            last_state = 0;
//        }
//        // else if(plane_type == 0)
//        // {
//        //   if(last_state == 1)
//        //   {
//        //     uint i_nex_tem;
//        //     uint j;
//        //     for(j=last_i+1; j<=last_i_nex; j++)
//        //     {
//        //       uint i_nex_tem2 = i_nex_tem;
//        //       Eigen::Vector3d curr_direct2;
//
//        //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);
//
//        //       if(ttem != 1)
//        //       {
//        //         i_nex_tem = i_nex_tem2;
//        //         break;
//        //       }
//        //       curr_direct = curr_direct2;
//        //     }
//
//        //     if(j == last_i+1)
//        //     {
//        //       last_state = 0;
//        //     }
//        //     else
//        //     {
//        //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
//        //       {
//        //         if(k != i_nex_tem)
//        //         {
//        //           types[k].ftype = Real_Plane;
//        //         }
//        //         else
//        //         {
//        //           types[k].ftype = Poss_Plane;
//        //         }
//        //       }
//        //       i = i_nex_tem-1;
//        //       i_nex = i_nex_tem;
//        //       i2 = j-1;
//        //       last_state = 1;
//        //     }
//
//        //   }
//        // }
//
//        last_i = i2;
//        last_i_nex = i_nex;
//        last_direct = curr_direct;
//    }
//
//    plsize2 = plsize > 3 ? plsize - 3 : 0;
//    for(uint i=head+3; i<plsize2; i++)
//    {
//        if(types[i].range<blind || types[i].ftype>=Real_Plane)
//        {
//            continue;
//        }
//
//        if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
//        {
//            continue;
//        }
//
//        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
//        Eigen::Vector3d vecs[2];
//
//        for(int j=0; j<2; j++)
//        {
//            int m = -1;
//            if(j == 1)
//            {
//                m = 1;
//            }
//
//            if(types[i+m].range < blind)
//            {
//                if(types[i].range > inf_bound)
//                {
//                    types[i].edj[j] = Nr_inf;
//                }
//                else
//                {
//                    types[i].edj[j] = Nr_blind;
//                }
//                continue;
//            }
//
//            vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
//            vecs[j] = vecs[j] - vec_a;
//
//            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
//            if(types[i].angle[j] < jump_up_limit)
//            {
//                types[i].edj[j] = Nr_180;
//            }
//            else if(types[i].angle[j] > jump_down_limit)
//            {
//                types[i].edj[j] = Nr_zero;
//            }
//        }
//
//        types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
//        if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
//        {
//            if(types[i].intersect > cos160)
//            {
//                if(edge_jump_judge(pl, types, i, Prev))
//                {
//                    types[i].ftype = Edge_Jump;
//                }
//            }
//        }
//        else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
//        {
//            if(types[i].intersect > cos160)
//            {
//                if(edge_jump_judge(pl, types, i, Next))
//                {
//                    types[i].ftype = Edge_Jump;
//                }
//            }
//        }
//        else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
//        {
//            if(edge_jump_judge(pl, types, i, Prev))
//            {
//                types[i].ftype = Edge_Jump;
//            }
//        }
//        else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
//        {
//            if(edge_jump_judge(pl, types, i, Next))
//            {
//                types[i].ftype = Edge_Jump;
//            }
//
//        }
//        else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
//        {
//            if(types[i].ftype == Nor)
//            {
//                types[i].ftype = Wire;
//            }
//        }
//    }
//
//    plsize2 = plsize-1;
//    double ratio;
//    for(uint i=head+1; i<plsize2; i++)
//    {
//        if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
//        {
//            continue;
//        }
//
//        if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
//        {
//            continue;
//        }
//
//        if(types[i].ftype == Nor)
//        {
//            if(types[i-1].dista > types[i].dista)
//            {
//                ratio = types[i-1].dista / types[i].dista;
//            }
//            else
//            {
//                ratio = types[i].dista / types[i-1].dista;
//            }
//
//            if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
//            {
//                if(types[i-1].ftype == Nor)
//                {
//                    types[i-1].ftype = Real_Plane;
//                }
//                if(types[i+1].ftype == Nor)
//                {
//                    types[i+1].ftype = Real_Plane;
//                }
//                types[i].ftype = Real_Plane;
//            }
//        }
//    }
//
//    int last_surface = -1;
//    for(uint j=head; j<plsize; j++)
//    {
//        if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
//        {
//            if(last_surface == -1)
//            {
//                last_surface = j;
//            }
//
//            if(j == uint(last_surface+point_filter_num-1))
//            {
//                PointType ap;
//                ap.x = pl[j].x;
//                ap.y = pl[j].y;
//                ap.z = pl[j].z;
//                ap.intensity = pl[j].intensity;
//                ap.curvature = pl[j].curvature;
//                pl_surf.push_back(ap);
//
//                last_surface = -1;
//            }
//        }
//        else
//        {
//            if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
//            {
//                pl_corn.push_back(pl[j]);
//            }
//            if(last_surface != -1)
//            {
//                PointType ap;
//                for(uint k=last_surface; k<j; k++)
//                {
//                    ap.x += pl[k].x;
//                    ap.y += pl[k].y;
//                    ap.z += pl[k].z;
//                    ap.intensity += pl[k].intensity;
//                    ap.curvature += pl[k].curvature;
//                }
//                ap.x /= (j-last_surface);
//                ap.y /= (j-last_surface);
//                ap.z /= (j-last_surface);
//                ap.intensity /= (j-last_surface);
//                ap.curvature /= (j-last_surface);
//                pl_surf.push_back(ap);
//            }
//            last_surface = -1;
//        }
//    }
//}
//
//double vx, vy, vz;
//void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
//{
//
//    PointCloudXYZI pl_full, pl_corn, pl_surf; //储存全部点(特征提取或间隔采样后）、角点、面特征点
//
//    pl_surf.clear();
//    pl_corn.clear();
//    pl_full.clear();
//
////    pcl::PointCloud<pcl::PointXYZ> pl_orig;
////    pcl::fromROSMsg(*msg, pl_orig);
//////    std::vector<int> indices;
//
//    pcl::PointCloud<velodyne_ros::Point> pl_orig;
//    pcl::fromROSMsg(*msg, pl_orig);
//    int plsize = pl_orig.points.size();
//    pl_surf.reserve(plsize);
//
//    /*** These variables only works when no point timestamps given ***/
//    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
//    std::vector<bool> is_first(N_SCANS,true);
//    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
//    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
//    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
//    /*****************************************************************/
//
//    if (pl_orig.points[plsize - 1].time > 0)
//    {
//        given_offset_time = true;
//    }
//    else
//    {
//        given_offset_time = false;
//        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
//        double yaw_end  = yaw_first;
//        int layer_first = pl_orig.points[0].ring;
//        for (uint i = plsize - 1; i > 0; i--)
//        {
//            if (pl_orig.points[i].ring == layer_first)
//            {
//                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
//                break;
//            }
//        }
//    }
//
//    if(feature_enabled)
//    {
//        for (int i = 0; i < N_SCANS; i++)
//        {
//            pl_buff[i].clear();
//            pl_buff[i].reserve(plsize);
//        }
//
//        for (int i = 0; i < plsize; i++)
//        {
//            PointType_XYZIN added_pt;
//            added_pt.normal_x = 0;
//            added_pt.normal_y = 0;
//            added_pt.normal_z = 0;
//            int layer  = pl_orig.points[i].ring;
//            if (layer >= N_SCANS) continue;
//            added_pt.x = pl_orig.points[i].x;
//            added_pt.y = pl_orig.points[i].y;
//            added_pt.z = pl_orig.points[i].z;
//            added_pt.intensity = pl_orig.points[i].intensity;
//            added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms
//
//            if (!given_offset_time)
//            {
//                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
//                if (is_first[layer])
//                {
//                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
//                    yaw_fp[layer]=yaw_angle;
//                    is_first[layer]=false;
//                    added_pt.curvature = 0.0;
//                    yaw_last[layer]=yaw_angle;
//                    time_last[layer]=added_pt.curvature;
//                    continue;
//                }
//
//                if (yaw_angle <= yaw_fp[layer])
//                {
//                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
//                }
//                else
//                {
//                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
//                }
//
//                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;
//
//                yaw_last[layer] = yaw_angle;
//                time_last[layer]=added_pt.curvature;
//            }
//
//            pl_buff[layer].points.push_back(added_pt);
//        }
//
//        for (int j = 0; j < N_SCANS; j++)
//        {
//            PointCloudXYZI &pl = pl_buff[j];
//            int linesize = pl.size();
//            if (linesize < 2) continue;
//            std::vector<orgtype> &types = typess[j];
//            types.clear();
//            types.resize(linesize);
//            linesize--;
//            for (uint i = 0; i < linesize; i++)
//            {
//                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
//                vx = pl[i].x - pl[i + 1].x;
//                vy = pl[i].y - pl[i + 1].y;
//                vz = pl[i].z - pl[i + 1].z;
//                types[i].dista = vx * vx + vy * vy + vz * vz;
//            }
//            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
//            give_feature(pl, types);
//        }
//    }
////    else
////    {
////        for (int i = 0; i < plsize; i++)
////        {
////            PointType_XYZIN added_pt;
////            // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
////
////            added_pt.normal_x = 0;
////            added_pt.normal_y = 0;
////            added_pt.normal_z = 0;
////            added_pt.x = pl_orig.points[i].x;
////            added_pt.y = pl_orig.points[i].y;
////            added_pt.z = pl_orig.points[i].z;
////            added_pt.intensity = pl_orig.points[i].intensity;
////            added_pt.curvature = pl_orig.points[i].time / 1000.0;  // curvature unit: ms
////
////            if (!given_offset_time)
////            {
////                int layer = pl_orig.points[i].ring;
////                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
////
////                if (is_first[layer])
////                {
////                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
////                    yaw_fp[layer]=yaw_angle;
////                    is_first[layer]=false;
////                    added_pt.curvature = 0.0;
////                    yaw_last[layer]=yaw_angle;
////                    time_last[layer]=added_pt.curvature;
////                    continue;
////                }
////
////                // compute offset time
////                if (yaw_angle <= yaw_fp[layer])
////                {
////                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
////                }
////                else
////                {
////                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
////                }
////
////                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;
////
////                yaw_last[layer] = yaw_angle;
////                time_last[layer]=added_pt.curvature;
////            }
////
////            if (i % point_filter_num == 0)
////            {
////                if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
////                {
////                    pl_surf.points.push_back(added_pt);
////                }
////            }
////        }
////    }
//}
/**********   from   fast-lio2     ***********/
/**********   from   fast-lio2     ***********/
/**********   from   fast-lio2     ***********/


//std::mutex mtx_buffer;
//std::condition_variable sig_buffer;
//int scan_count = 0;
//double last_timestamp_lidar = 0;
//std::deque<PointCloudXYZI::Ptr>  lidar_buffer; //记录特征提取或间隔采样后的lidar（特征）数据
shared_ptr<Preprocess> p_pre(new Preprocess());
//std::deque<double> time_buffer;
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    {
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

//    TicToc t_whole;
//    TicToc t_prepare;
//    std::vector<int> scanStartInd(N_SCANS, 0);
//    std::vector<int> scanEndInd(N_SCANS, 0);
//
//    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
//    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
//    std::vector<int> indices;
//
//
//    // 首先对点云滤波，去除NaN值得无效点云，以及在Lidar坐标系原点MINIMUM_RANGE距离以内的点
//    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
//    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);
//
//
//    int cloudSize = laserCloudIn.points.size();
//    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
//    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
//                          laserCloudIn.points[cloudSize - 1].x) +
//                   2 * M_PI;
//
//    if (endOri - startOri > 3 * M_PI)
//    {
//        endOri -= 2 * M_PI;
//    }
//    else if (endOri - startOri < M_PI)
//    {
//        endOri += 2 * M_PI;
//    }
//    //printf("end Ori %f\n", endOri);
//
//    bool halfPassed = false;
//    int count = cloudSize;
//    PointType point;
//    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
//    for (int i = 0; i < cloudSize; i++)
//    {
//        point.x = laserCloudIn.points[i].x;
//        point.y = laserCloudIn.points[i].y;
//        point.z = laserCloudIn.points[i].z;
//
//        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
//        int scanID = 0;
//
//        if (N_SCANS == 16)
//        {
//            scanID = int((angle + 15) / 2 + 0.5);
//            if (scanID > (N_SCANS - 1) || scanID < 0)
//            {
//                count--;
//                continue;
//            }
//        }
//        else if (N_SCANS == 32)
//        {
//            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
//            if (scanID > (N_SCANS - 1) || scanID < 0)
//            {
//                count--;
//                continue;
//            }
//        }
//        else if (N_SCANS == 64)
//        {
//            if (angle >= -8.83)
//                scanID = int((2 - angle) * 3.0 + 0.5);
//            else
//                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
//
//            // use [0 50]  > 50 remove outlies
//            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
//            {
//                count--;
//                continue;
//            }
//        }
//        else
//        {
//            printf("wrong scan number\n");
//            ROS_BREAK();
//        }
//        //printf("angle %f scanID %d \n", angle, scanID);
//
//        float ori = -atan2(point.y, point.x);
//        if (!halfPassed)
//        {
//            if (ori < startOri - M_PI / 2)
//            {
//                ori += 2 * M_PI;
//            }
//            else if (ori > startOri + M_PI * 3 / 2)
//            {
//                ori -= 2 * M_PI;
//            }
//
//            if (ori - startOri > M_PI)
//            {
//                halfPassed = true;
//            }
//        }
//        else
//        {
//            ori += 2 * M_PI;
//            if (ori < endOri - M_PI * 3 / 2)
//            {
//                ori += 2 * M_PI;
//            }
//            else if (ori > endOri + M_PI / 2)
//            {
//                ori -= 2 * M_PI;
//            }
//        }
//
//        float relTime = (ori - startOri) / (endOri - startOri);//点在点云中的相对时间
//        point.intensity = scanID + scanPeriod * relTime;
////        ROS_INFO("intensity: %f", point.intensity);
//
//        laserCloudScans[scanID].push_back(point);
//    }
//
//    cloudSize = count;
//    printf("points size %d \n", cloudSize);
//
//    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
//    for (int i = 0; i < N_SCANS; i++)
//    {
//        scanStartInd[i] = laserCloud->size() + 5;// 记录每个scan的开始index，忽略前5个点
//        *laserCloud += laserCloudScans[i];
//        scanEndInd[i] = laserCloud->size() - 6;// // 记录每个scan的结束index，忽略后5个点，开始和结束处的点云scan容易产生不闭合的“接缝”，对提取edge feature不利
//    }
//
//    printf("prepare time %f \n", t_prepare.toc());
//
//    for (int i = 5; i < cloudSize - 5; i++)
//    {
//        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
//        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
//        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
//
//        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
//        cloudSortInd[i] = i;
//        cloudNeighborPicked[i] = 0;// 点有没有被选选择为feature点
//        cloudLabel[i] = 0;// Label 2: corner_sharp
//                          // Label 1: corner_less_sharp, 包含Label 2
//                          // Label -1: surf_flat
//                          // Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
//    }
//
//
//    TicToc t_pts;
//
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    //todo feature extraction from fast-lio2

    //////////
    PointCloudXYZI::Ptr  surf_ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  corn_ptr(new PointCloudXYZI());
    p_pre->process(laserCloudMsg, surf_ptr, corn_ptr);
//    ROS_INFO("ptr.size(): %d", ptr->size());

    int num_surf_pts = static_cast<int>(surf_ptr->size());
    ROS_INFO("number surface points: %d", num_surf_pts);
    int num_corner_pts = static_cast<int>(corn_ptr->size());
    ROS_INFO("number corner points: %d", num_corner_pts);

    plane_feature_size.emplace_back(num_surf_pts);
    edge_feature_size.emplace_back(num_corner_pts);

    surfPointsLessFlat.resize(num_surf_pts);
    for (int i = 0; i < num_surf_pts; ++i) {
        surfPointsLessFlat.points[i].x = surf_ptr->points[i].x;
        surfPointsLessFlat.points[i].y = surf_ptr->points[i].y;
        surfPointsLessFlat.points[i].z = surf_ptr->points[i].z;
        surfPointsLessFlat.points[i].intensity = surf_ptr->points[i].intensity;
    }

    surfPointsFlat.resize(num_surf_pts);
    for (int i = 0; i < num_surf_pts; ++i) {
        surfPointsFlat.points[i].x = surf_ptr->points[i].x;
        surfPointsFlat.points[i].y = surf_ptr->points[i].y;
        surfPointsFlat.points[i].z = surf_ptr->points[i].z;
        surfPointsFlat.points[i].intensity = surf_ptr->points[i].intensity;
    }

    cornerPointsLessSharp.resize(num_corner_pts);
    for (int i = 0; i < num_corner_pts; ++i) {
        cornerPointsLessSharp.points[i].x = corn_ptr->points[i].x;
        cornerPointsLessSharp.points[i].y = corn_ptr->points[i].y;
        cornerPointsLessSharp.points[i].z = corn_ptr->points[i].z;
        cornerPointsLessSharp.points[i].intensity = corn_ptr->points[i].intensity;
    }

    cornerPointsSharp.resize(num_corner_pts);
    for (int i = 0; i < num_corner_pts; ++i) {
        cornerPointsSharp.points[i].x = corn_ptr->points[i].x;
        cornerPointsSharp.points[i].y = corn_ptr->points[i].y;
        cornerPointsSharp.points[i].z = corn_ptr->points[i].z;
        cornerPointsSharp.points[i].intensity = corn_ptr->points[i].intensity;
    }
    /////////


    float t_q_sort = 0;
//    for (int i = 0; i < N_SCANS; i++)// 按照scan的顺序提取4种特征点
//    {
//        if( scanEndInd[i] - scanStartInd[i] < 6)// 如果该scan的点数少于7个点，就跳过
//            continue;
//        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
//        for (int j = 0; j < 6; j++)// 将该scan分成6小段执行特征检测
//        {
//            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;// subscan的起始index
//            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;// subscan的结束index
//
//            TicToc t_tmp;
//            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);// 根据曲率有小到大对subscan的点进行sort
//            t_q_sort += t_tmp.toc();
//
//            int largestPickedNum = 0;
//            for (int k = ep; k >= sp; k--)// 从后往前，即从曲率大的点开始提取corner feature
//            {
//                int ind = cloudSortInd[k];
//
//                if (cloudNeighborPicked[ind] == 0 &&
//                    cloudCurvature[ind] > 0.1)// 如果该点没有被选择过，并且曲率大于0.1
//                {
//                    largestPickedNum++;
//                    if (largestPickedNum <= 2)// 该subscan中曲率最大的前2个点认为是corner_sharp特征点
//                    {
//                        cloudLabel[ind] = 2;
//                        cornerPointsSharp.push_back(laserCloud->points[ind]);
//                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
//                    }
//                    else if (largestPickedNum <= 20)// 该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
//                    {
//                        cloudLabel[ind] = 1;
//                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
//                    }
//                    else
//                    {
//                        break;
//                    }
//
//                    cloudNeighborPicked[ind] = 1;// 已被标记
//
//                    // 与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
//                    for (int l = 1; l <= 5; l++)
//                    {
//                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
//                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
//                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
//                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
//                        {
//                            break;
//                        }
//
//                        cloudNeighborPicked[ind + l] = 1;
//                    }
//                    for (int l = -1; l >= -5; l--)
//                    {
//                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
//                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
//                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
//                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
//                        {
//                            break;
//                        }
//
//                        cloudNeighborPicked[ind + l] = 1;
//                    }
//                }
//            }
//
//            // 提取surf平面feature，与上述类似，选取该subscan曲率最小的前4个点为surf_flat
//            int smallestPickedNum = 0;
//            for (int k = sp; k <= ep; k++)
//            {
//                int ind = cloudSortInd[k];
//
//                if (cloudNeighborPicked[ind] == 0 &&
//                    cloudCurvature[ind] < 0.1)
//                {
//
//                    cloudLabel[ind] = -1;
//                    surfPointsFlat.push_back(laserCloud->points[ind]);
//
//                    smallestPickedNum++;
//                    if (smallestPickedNum >= 4)
//                    {
//                        break;
//                    }
//
//                    cloudNeighborPicked[ind] = 1;
//                    for (int l = 1; l <= 5; l++)
//                    {
//                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
//                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
//                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
//                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
//                        {
//                            break;
//                        }
//
//                        cloudNeighborPicked[ind + l] = 1;
//                    }
//                    for (int l = -1; l >= -5; l--)
//                    {
//                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
//                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
//                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
//                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
//                        {
//                            break;
//                        }
//
//                        cloudNeighborPicked[ind + l] = 1;
//                    }
//                }
//            }
//
//
//            // 其他的非corner特征点与surf_flat特征点一起组成surf_less_flat特征点
//            for (int k = sp; k <= ep; k++)
//            {
//                if (cloudLabel[k] <= 0)
//                {
//                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
//                }
//            }
//        }
//
//        // 最后对该scan点云中提取的所有surf_less_flat特征点进行降采样，因为点太多了
//        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
//        pcl::VoxelGrid<PointType> downSizeFilter;
//        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
//        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//        downSizeFilter.filter(surfPointsLessFlatScanDS);
//
//        surfPointsLessFlat += surfPointsLessFlatScanDS;
//    }
//    printf("sort q time %f \n", t_q_sort);
//    printf("seperate points time %f \n", t_pts.toc());


//    sensor_msgs::PointCloud2 laserCloudOutMsg;
//    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
//    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
//    laserCloudOutMsg.header.frame_id = "/camera_init";
//    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

//    // pub each scam
//    if(PUB_EACH_LINE)
//    {
//        for(int i = 0; i< N_SCANS; i++)
//        {
//            sensor_msgs::PointCloud2 scanMsg;
//            pcl::toROSMsg(laserCloudScans[i], scanMsg);
//            scanMsg.header.stamp = laserCloudMsg->header.stamp;
//            scanMsg.header.frame_id = "/camera_init";
//            pubEachScan[i].publish(scanMsg);
//        }
//    }

//    printf("scan registration time %f ms *************\n", t_whole.toc());
//    if(t_whole.toc() > 100)
//        ROS_WARN("scan registration process over 100ms");
}



//void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
//{
//    mtx_buffer.lock();
//    scan_count ++;
////    double preprocess_start_time = omp_get_wtime();
//    if (msg->header.stamp.toSec() < last_timestamp_lidar)
//    {
//        ROS_ERROR("lidar loop back, clear buffer");
//        lidar_buffer.clear();
//    }
//
//    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
//    p_pre->process(msg, ptr);
//    lidar_buffer.push_back(ptr);
//    time_buffer.push_back(msg->header.stamp.toSec());
//    last_timestamp_lidar = msg->header.stamp.toSec();
//    ROS_INFO("lidar_buffer.size: %d", lidar_buffer.size());
////    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
//    mtx_buffer.unlock();
//    sig_buffer.notify_all();
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    std::string topic_cloud_in;
    nh.param<std::string>("topic_cloud_in",topic_cloud_in,"/velodyne_points");
    nh.param<double>("preprocess/blind", p_pre->blind, 2.0);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, 2);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 32);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, true);


    //将名为 scan_line 的变量赋值给N_SCANS，没有参数则设动为16
    nh.param<int>("scan_line", N_SCANS, 16);

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64 && N_SCANS != 128)
    {
        printf("only support velodyne with 16, 32, 64 or 128 scan line!");
        return 0;
    }


    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_cloud_in, 100, laserCloudHandler);
//    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_cloud_in, 100, standard_pcl_cbk);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    {
        float plane_feature_aveg = 0;
        for (const size_t &a: plane_feature_size) {
            plane_feature_aveg += static_cast<float>(a) / static_cast<float>(plane_feature_size.size());
        }
        printf("\033[1;32m scanRegistration plane_feature_aveg: %f\033[0m\n", plane_feature_aveg);
    }
    {
        float edge_feature_aveg = 0;
        for (const size_t &a: edge_feature_size) {
            edge_feature_aveg += static_cast<float>(a) / static_cast<float>(edge_feature_size.size());
        }
        printf("\033[1;32m scanRegistration edge_feature_aveg: %f\033[0m\n", edge_feature_aveg);
    }

    return 0;
}
