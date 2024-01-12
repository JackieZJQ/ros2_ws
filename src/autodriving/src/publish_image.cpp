#include "rclcpp/rclcpp.hpp"

// 读取文件流
#include <iostream>
#include <fstream>
#include <iomanip>

#include <string>

// string流 image_id: int -> string
#include <sstream>

// 图像相关
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>

// 点云相关
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

// imu
#include "sensor_msgs/msg/imu.hpp"

// gps
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// Marker
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// 旋转
#include "tf2_eigen/tf2_eigen.h"

// 时钟
#include <chrono>

#include <map>

#include <Eigen/Core>

#include <cmath>

// struct imuStruct {
//     float lat;   
//     float lon;   
//     float alt;   
//     float roll;  
//     float pitch; 
//     float yaw;   
//     float vn;    
//     float ve;   
//     float vf;    
//     float vl;   
//     float vu;    
//     float ax;    
//     float ay;    
//     float ay;    
//     float af;    
//     float al;    
//     float au;    
//     float wx;    
//     float wy;   
//     float wz;    
//     float wf;   
//     float wl;    
//     float wu;    
//     float pos_accuracy;  
//     float vel_accuracy;  
//     int navstat;       
//     int numsats;       
//     int posmode;       
//     int velmode;     
//     int orimode; 
// };

void readPointcloud(const std::string &path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    cloud->clear();
    
    pcl::PointXYZ point;
    float r = 0;
    
    std::ifstream binfile(path.c_str(), std::ios::in | std::ios::binary);
    if(!binfile.good()) {
        std::cout << "open bin file fails!!" << std::endl;
    }

    for(int i=0; binfile.good()&&!binfile.eof(); i++) {
        binfile.read((char *)&point.x, 3*sizeof(float));
        binfile.read((char *)&r, sizeof(float));
        cloud->push_back(point);
    }

    binfile.close();
}

void readImu(const std::string &path, float num[]) {

    std::ifstream txtfile(path.c_str(), std::ios::in);
    if (!txtfile.good()) {
        std::cout << "open txt file fails!!" << std::endl;
    }
    
    for (int i=0; txtfile.good()&&!txtfile.eof(); i++) {
        txtfile >> num[i];
    }

    // imuStruct imuData;
    // for (int i = 0; i < 30; i++) {
    //     txtfile >> num[i];
    // }
    // std::cout << num[14] << std::endl;
    // std::cout << num[15] << std::endl;
    // std::cout << num[16] << std::endl;

    txtfile.close();
}

void readTracking(const std::string &path, std::vector<std::vector<float>> &trackingData, std::vector<std::string> &type) {

    std::ifstream txtfile(path.c_str(), std::ios::in);
    if (!txtfile.good()) {
        std::cout << "open txt file fails!!" << std::endl;
    }

    for (int i=0; txtfile.good()&&!txtfile.eof(); i++) {

        std::vector<float> data;
        trackingData.push_back(data);
        
        // 1.
        float num;
        txtfile >> num;
        trackingData[i].push_back(num);

        // 2.
        txtfile >> num;
        trackingData[i].push_back(num);

        // 3. 
        char temp[32];
        txtfile >> temp;
        type.push_back(temp);

        // 循环读剩余数字
        for (int cnt=2; cnt<=15; cnt++) {
            float tempnum2;
            txtfile >> tempnum2;
            trackingData[i].push_back(tempnum2);
        }
    }

    txtfile.close();

    // 处理种类
    for (size_t i = 0; i < type.size(); i++) {
        if (type[i] == "Truck" || type[i] == "Van" || type[i] == "Tram") {
            type[i] = "Car";
        }
    }
}


void declareCarsight(visualization_msgs::msg::MarkerArray &markerArray) {

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    // marker.header.stamp = rclcpp::Clock().now(); // ?
    
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(std::chrono::seconds::max());
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.scale.x = 0.1;

    geometry_msgs::msg::Point p1;
    p1.x = 10; p1.y = 10; p1.z = 0;
    geometry_msgs::msg::Point p2;
    p2.x = 0; p2.y = 0; p2.z = 0;
    geometry_msgs::msg::Point p3;
    p3.x = 10; p3.y = -10; p3.z = 0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);

    markerArray.markers.push_back(marker);
}

void declareCarmodel(visualization_msgs::msg::MarkerArray &markerArray) {
    
    visualization_msgs::msg::Marker mesh_marker;

    mesh_marker.header.frame_id = "map";
    // mesh_marker.header.stamp = rclcpp::Clock().now(); // ?

    mesh_marker.id = -1;
    mesh_marker.lifetime = rclcpp::Duration(std::chrono::seconds::max());
    mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_marker.mesh_resource = "file:/home/jackie/Softwares/ros2_ws/src/autodriving/bmw_x5/BMW X5 4.dae"; // 需要在cmakelist中配置install才可使用package

    mesh_marker.pose.position.x = 0.0;
    mesh_marker.pose.position.y = 0.0;
    mesh_marker.pose.position.z = -1.73;

    tf2::Quaternion q;
    q.setEuler(M_PI, -M_PI/2, 0);

    mesh_marker.pose.orientation.x = q[0];
    mesh_marker.pose.orientation.y = q[1]; 
    mesh_marker.pose.orientation.z = q[2]; 
    mesh_marker.pose.orientation.w = q[3]; 
 
    mesh_marker.color.r = 1.0;
    mesh_marker.color.g = 1.0;
    mesh_marker.color.b = 1.0;
    mesh_marker.color.a = 1.0;

    mesh_marker.scale.x = 0.9;
    mesh_marker.scale.y = 0.9;
    mesh_marker.scale.z = 0.9;

    markerArray.markers.push_back(mesh_marker);
}

void declareImu(sensor_msgs::msg::Imu &imu, float imuGpsData[]) {

    imu.header.frame_id = "map";
    imu.header.stamp = rclcpp::Clock().now();

    // 设定旋转
    tf2::Quaternion q;
    q.setEuler(imuGpsData[5], imuGpsData[4], imuGpsData[3]);
    imu.orientation.x = q[0];
    imu.orientation.y = q[1];
    imu.orientation.z = q[2];
    imu.orientation.w = q[3];

    // 设定线性加速度
    imu.linear_acceleration.x = imuGpsData[14];
    imu.linear_acceleration.y = imuGpsData[15];
    imu.linear_acceleration.z = imuGpsData[16];

    // 设定角速度
    imu.angular_velocity.x = imuGpsData[20];
    imu.angular_velocity.y = imuGpsData[21];
    imu.angular_velocity.z = imuGpsData[22];
}

void declareGps(sensor_msgs::msg::NavSatFix &gps, float imuGpsData[]) {
    
    gps.header.frame_id = "map";
    gps.header.stamp = rclcpp::Clock().now();

    gps.latitude = imuGpsData[0];
    gps.longitude = imuGpsData[1];
    gps.altitude = imuGpsData[2];
}

Eigen::Matrix<float, 3, 8> compute_3d_box_cam2(float x, float y, float z, float l, float w, float h, float yaw) {

    Eigen::Matrix3f R;
    R <<  std::cos(yaw), 0, std::sin(yaw),
                      0, 1, 0,
         -std::sin(yaw), 0, std::cos(yaw);
         
    Eigen::Matrix<float, 3, 8> corners;
    corners << l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2,
                 0,   0,  0,   0,  -h,-h,  -h,  -h,
               w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2;
    
    Eigen::Matrix<float, 3, 8> corners_3d_cam2 = R * corners;

    Eigen::Vector3f pos;
    pos << x, y, z;

    corners_3d_cam2.colwise() += pos;

    return corners_3d_cam2;
}


int main(int argc, char **argv) {

    std::string path = "/home/jackie/Softwares/KITTI/RawData/2011_09_26/2011_09_26_drive_0005_sync/";

    int cnt = 0;

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("kitti_publisher", options);
    
    image_transport::ImageTransport it(node);
    image_transport::Publisher pub = it.advertise("kitti/cam", 1);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/pointcloud", 5);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 5);
    
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher = node->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/gps", 5);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/careyesight", 5);


    // 声明图片发布数据
    cv::Mat frame;
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;

    // 声明点云发布数据
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 pointCloudmsg;
    
    // 声明照相机视野Marker
    visualization_msgs::msg::MarkerArray markerArray;
    declareCarsight(markerArray);
    declareCarmodel(markerArray);

    // 声明IMU和GPS
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::NavSatFix gps;
    float imuGpsData[30];

    // 读取tracking数据
    std::string labelPath = "/home/jackie/Softwares/KITTI/training/label_02/0000.txt";
    std::vector<std::vector<float>> trackingData;
    std::vector<std::string> trackingLabel;
    readTracking(labelPath, trackingData, trackingLabel);

    // 指定tracking里的指针，这样就不用循环遍历
    int txtId = 0;
    
    // 定义不用种类的颜色
    std::map<std::string, cv::Scalar> DETECTION_COLOR_DICT = {
        {"Car", cv::Scalar(255, 255, 0)},
        {"Cyclist", cv::Scalar(141, 40, 255)},
        {"Pedestrian", cv::Scalar(0, 226, 255)}};

    // cv::Mat testImage = cv::imread("/home/jackie/Softwares/KITTI/RawData/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/0000000000.png", cv::IMREAD_COLOR);
    // cv::rectangle(testImage, cv::Point(trackingData[2][5], trackingData[2][6]), cv::Point(trackingData[2][7], trackingData[2][8]), cv::Scalar(255, 255, 0), 1);
    // int tt = 0;
    // for (;txtId < (int)trackingData.size()&&trackingData[txtId][0]==tt; txtId++) {
    //     if (trackingLabel[txtId] == "DontCare") continue;
    //     cv::rectangle(testImage, cv::Point(trackingData[txtId][5], trackingData[txtId][6]), cv::Point(trackingData[txtId][7], trackingData[txtId][8]), DETECTION_COLOR_DICT[trackingLabel[txtId]], 1);
    // }

    // cv::imshow("Test Pic", testImage);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok()) {
        
        // 计算帧id
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(10) << cnt;
        std::string frameId;
        ss >> frameId;
        
        ////////////////////////读取并发布图片///////////////////////////////
        std::string imagePath = path+"image_02/data/"+frameId+".png";
        frame = cv::imread(imagePath, cv::IMREAD_COLOR);

        for (;txtId < (int)trackingData.size()&&trackingData[txtId][0]==cnt; txtId++) {
            if (trackingLabel[txtId] == "DontCare") continue;
            cv::rectangle(frame, cv::Point(trackingData[txtId][5], trackingData[txtId][6]), cv::Point(trackingData[txtId][7], trackingData[txtId][8]), DETECTION_COLOR_DICT[trackingLabel[txtId]], 2);
        }
        
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
        msg->header.stamp = rclcpp::Clock().now();
        pub.publish(msg);
        
        ////////////////////////读取并发布点云///////////////////////////////
        std::string cloudPath = path+"velodyne_points/data/"+frameId+".bin";
        readPointcloud(cloudPath, cloud);
        
        pcl::toROSMsg(*cloud, pointCloudmsg);
        pointCloudmsg.header.frame_id = "map";
        pointCloudmsg.header.stamp = rclcpp::Clock().now();
        cloud_publisher->publish(pointCloudmsg);

        ////////////////////////读取并发布IMU 和 GPS///////////////////////////
        std::string imuGpsPath = path+"oxts/data/"+frameId+".txt";
        readImu(imuGpsPath, imuGpsData);
        declareImu(imu, imuGpsData);
        imu_publisher->publish(imu);

        declareGps(gps, imuGpsData);
        gps_publisher->publish(gps);

        ////////////////////////发布相机视野与汽车模型///////////////////////////
        marker_publisher->publish(markerArray);

        RCLCPP_INFO(node->get_logger(), "published");
        rclcpp::spin_some(node);
        loop_rate.sleep();

        // 更新读取数据的下标顺序
        cnt++;
        cnt = cnt % 154;
        txtId = txtId % 1089;
    }

    return 0;
}

// 发布imu数据时并未校准至点云坐标系