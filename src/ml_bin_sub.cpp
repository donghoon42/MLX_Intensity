#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include "ml/libsoslab_ml.h"
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>

typedef pcl::PointXYZRGB PointRGB_T;
typedef pcl::PointCloud<PointRGB_T> PointCloud_T;

// int nCols = 0;
// int nRows = 0;

// unsigned char soslab_r[] = {3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 41, 42, 42, 43, 43, 43, 44, 44, 45, 45, 46, 46, 46, 47, 47, 48, 48, 49, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 57, 57, 57, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 62, 63, 63, 64, 64, 65, 65, 66, 66, 66, 67, 67, 68, 68, 70, 71, 73, 75, 77, 79, 80, 82, 84, 86, 88, 90, 92, 93, 95, 97, 99, 101, 102, 104, 106, 108, 110, 111, 113, 115, 117, 119, 120, 122, 124, 126, 128, 129, 131, 133, 135, 137, 138, 140, 142, 144, 145, 147, 149, 151, 153, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 172, 174, 176, 178, 180, 181, 183, 184, 185, 187, 188, 189, 190, 191, 192, 194, 195, 196, 197, 198, 199, 200, 201, 203, 204, 205, 206, 207, 208, 209, 210, 212, 213, 214, 215, 216, 217, 218, 219, 221, 222, 223, 224, 225, 226, 227, 228, 230, 231, 232, 233, 234, 235, 236, 237, 239, 240, 241, 242, 243, 244, 245, 246, 248, 249, 250, 251, 252, 253, 254, 254};
// unsigned char soslab_g[] = {18, 19, 21, 23, 24, 26, 27, 29, 30, 32, 33, 35, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 55, 57, 58, 60, 61, 63, 64, 66, 67, 69, 71, 72, 74, 75, 77, 78, 80, 81, 83, 84, 86, 88, 89, 91, 92, 94, 95, 97, 98, 100, 101, 103, 105, 106, 108, 109, 111, 112, 114, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 171, 172, 173, 174, 175, 176, 177, 178, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200, 201, 202, 203, 204, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 247, 248, 249, 250, 251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
// unsigned char soslab_b[] = {107, 108, 109, 110, 111, 112, 113, 114, 114, 115, 116, 117, 118, 119, 120, 120, 121, 122, 123, 124, 125, 126, 126, 127, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 138, 138, 139, 140, 141, 142, 143, 144, 144, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157, 157, 158, 159, 160, 161, 162, 162, 163, 163, 163, 164, 164, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 167, 168, 168, 168, 169, 169, 169, 169, 170, 170, 170, 171, 171, 171, 172, 172, 172, 173, 173, 173, 174, 174, 174, 174, 175, 175, 175, 176, 176, 176, 177, 177, 177, 178, 178, 178, 179, 179, 179, 179, 180, 180, 180, 181, 181, 181, 182, 182, 181, 180, 179, 178, 177, 175, 175, 174, 172, 171, 170, 169, 168, 167, 166, 165, 164, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123, 121, 120, 119, 118, 117, 116, 115, 114, 113, 111, 109, 107, 105, 104, 102, 100, 98, 96, 95, 93, 91, 89, 88, 86, 84, 82, 81, 79, 77, 75, 73, 72, 70, 68, 66, 65, 63, 61, 59, 58, 56, 54, 52, 51, 49, 47, 45, 43, 42, 40, 38, 36, 35, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 15, 13, 12, 10, 8, 6, 5, 2, 1, 1};


// cv::Mat colormap(cv::Mat image)
// {
//     nCols = image.cols;
//     nRows = image.rows;
    
//     int temp_intensity = 0;

//     cv::Mat rgb_image(nRows, nCols, CV_8UC3);

//     for (int col = 0; col < nCols; col++){
//         for (int row = 0; row < nRows; row++){
//             temp_intensity = image.at<uint8_t>(row, col);
//             float colormap_val0, colormap_val1, colormap_val2;
            
//             colormap_val0 = soslab_r[temp_intensity];
//             colormap_val1 = soslab_g[temp_intensity];
//             colormap_val2 = soslab_b[temp_intensity];

//             rgb_image.at<cv::Vec3b>(row, col)[0] = int(colormap_val0);
//             rgb_image.at<cv::Vec3b>(row, col)[1] = int(colormap_val1);
//             rgb_image.at<cv::Vec3b>(row, col)[2] = int(colormap_val2);
//         }
//     }
//     return rgb_image;
// }

class ML_bin_subscriber
{
public:
    ML_bin_subscriber();
    ~ML_bin_subscriber();
    void InitROS();
    void publish_pc();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{"~"};
    std::string frame_id;
    ros::Publisher publisher_pc;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_vec;
    std::string bin_file;
    std::string output_topicname;
    double publish_rate=20.0;
    bool loop=false;
    std::shared_ptr<SOSLAB::LidarML> lidar_ml = std::make_shared<SOSLAB::LidarML>();
};

ML_bin_subscriber::ML_bin_subscriber()
{
    InitROS();
}
ML_bin_subscriber::~ML_bin_subscriber()
{
}
void ML_bin_subscriber::InitROS()
{
    nh_priv.param<std::string>("frame_id", frame_id, frame_id);
    nh_priv.param<std::string>("bin_file", bin_file, bin_file);
    nh_priv.param<std::string>("output_topicname", output_topicname, output_topicname);
    nh_priv.param<double>("publish_rate", publish_rate, publish_rate);
    nh_priv.param<bool>("loop", loop, loop);
    publisher_pc = nh.advertise<sensor_msgs::PointCloud2>(output_topicname, 1);

    std::ifstream file;
    file.open(bin_file, std::ios::in);
    if ( ! file.is_open() ) {                 
        std::cout << "\n[ERROR] bin file is wrong!" << std::endl;
        ros::shutdown();

        return;
    }
    else{
        file.close();
    }
    if(bin_file.empty()==true) {
        std::cout << "\n[ERROR] bin file is wrong!" << std::endl;
        ros::shutdown();
    
        return;
    }

}
void ML_bin_subscriber::publish_pc(){
    bool retval = lidar_ml->play_start(bin_file);
    if(!retval){
        std::cout << "[ERROR] Failed to play start" << std::endl;
        ros::shutdown();
    
        return;
    }
    uint64_t total_num_frame = 0;

    int mil_sec = 1000/publish_rate;

    if(lidar_ml->get_file_size(total_num_frame)){
        boost::filesystem::path path(bin_file);
        std::string filename = path.stem().string();
        std::string parent_path = path.parent_path().string();

        std::cout << "File name : " << filename << std::endl;
        std::cout << "Parent path : " << parent_path << std::endl;

        SOSLAB::LidarML::scene_t scene;
        
        for(uint64_t i=0; i<total_num_frame; i++){
            if(lidar_ml->get_scene(scene, i)){        
                // std::cout << "scene num : " << (i+1) << "/" << total_num_frame << std::endl;
                std::vector<SOSLAB::point_t> pointcloud = scene.pointcloud[0];

                std::size_t height = scene.rows;	// Lidar frame의 height 정보입니다.
                std::size_t width = scene.cols;	// Lidar frame의 width 정보입니다.
                std::size_t width2 = (scene.cols == 192) ? scene.cols*3 : scene.cols;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                msg_pointcloud->header.frame_id = "map";
                msg_pointcloud->width = width;
                msg_pointcloud->height = height;
                msg_pointcloud->points.resize(pointcloud.size());

                for (int col=0; col < width; col++) {
                    for (int row = 0; row < height; row++) {
                        int idx = col + (width * row);

                        //unit : (m)
                        msg_pointcloud->points[idx].x = pointcloud[idx].x / 1000.0;
                        msg_pointcloud->points[idx].y = pointcloud[idx].y / 1000.0;
                        msg_pointcloud->points[idx].z = pointcloud[idx].z / 1000.0;
                    }
                }
                msg_pointcloud->header.frame_id = frame_id;
                publisher_pc.publish(msg_pointcloud);
                pc_vec.push_back(msg_pointcloud);
            }


            std::this_thread::sleep_for(std::chrono::milliseconds(mil_sec));
        }
    }

    if(loop){
        while(ros::ok()){
            for(int i=0; i<pc_vec.size(); i++){
                if(!ros::ok()){
                    break;
                }
                publisher_pc.publish(pc_vec[i]);
                std::this_thread::sleep_for(std::chrono::milliseconds(mil_sec));
            }
        }
    }
    pc_vec.clear();
    return;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "ml_bin_subscriber");
    ML_bin_subscriber ml_bin_subscriber;
    ml_bin_subscriber.publish_pc();

    ros::shutdown();

    return 0;
}
