
#pragma warning(disable:4996);

// ROS
#include <ros/ros.h>
//

// D435i
#include <librealsense2/rs.hpp>
//

#include <csignal>
#include <iostream>
#include <map> // used for hashmap to give certainty
#include <vector> // used in hashmap
#include <numeric> // used for summing a vector
#include <stdio.h>


// ROS sensor messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

// ROS image geometry
#include <image_geometry/pinhole_camera_model.h>

// ROS transform
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <alfons_msgs/ArucoInfo.h>

// ROS CvBridge
#include <cv_bridge/cv_bridge.h>

// Image Transport to publish output img
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/ocl.hpp> // for opencl
//
#include <CL/cl.hpp>
//
#include "fdcl_common.hpp"

//

using namespace cv;
using namespace std;
using namespace sensor_msgs;

// OpenCL 커널 소스 코드
const char* kernelSource = R"(
__kernel void rgb_to_gray(__global unsigned char* inputImage, __global unsigned char* outputImage, int width, int height) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if (x < width && y < height) {
        int idx = (y * width + x) * 3;
        unsigned char r = inputImage[idx];
        unsigned char g = inputImage[idx + 1];
        unsigned char b = inputImage[idx + 2];
        outputImage[y * width + x] = (r + g + b) / 3;
    }
}
)";

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{ 6, 9 };

// Publisher
image_transport::Publisher result_img_pub_;
ros::Publisher tf_list_pub_;
ros::Publisher aruco_info_pub_;
ros::Publisher trans_Vec_pub_;


#define SSTR(x) static_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()
#define ROUND2(x) std::round(x * 100) / 100 //round :: 반올림, ceil :: 올림, floor :: 내림
#define ROUND3(x) std::round(x * 1000) / 1000

// Define global variables
bool camera_model_computed = false;
bool show_detections;
float marker_size=0.0218f;
float marker2_size=0.50f;


image_geometry::PinholeCameraModel camera_model;


// Mat distortion_coefficients; // define matrix, Mat() :: 2x2 matrix


// Matx33d intrinsic_matrix; // define matrix, 3x3 matrix - double
/*
Extrinsic Parameter는 3D 공간 내에서 카메라가 어디에 위치(3D Translation)하고 있고, 
어디를 바라보고 있는지(3D Rotation)에 대한 Parameter입니다.

Intrinsic Parameter는 카메라 렌즈와 센서 위치에 의해서 결정되어지는 항목으로써, 이미지 패널이 얼마나 이동(2D Translation)하고, 
얼마나 확대하고(2D Scaling), 얼마나 기울어졌는지(2D Shear) 대한 Parameter입니다.

Intrinsic은 카메라 박스 내에서 조절하는 parameter를, Extrinsic은 그 외적인 parameter라고 생각하시면 됩니다.
*/

Ptr<aruco::DetectorParameters> detector_params; 
Ptr<cv::aruco::Dictionary> dictionary;
string marker_tf_prefix;
int blur_window_size = 7;
int image_fps = 30;
int image_width = 640;
int image_height = 480;
bool enable_blur = true;

// hashmap used for uncertainty:
int num_detected = 10;  // =0 -> not used
int min_prec_value = 95; // min precentage value to be a detected marker.
map<int,  std::vector<int>  > ids_hashmap;   // key: ids, value: number within last 100 imgs
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Kalman filter variables
cv::KalmanFilter kf(6, 3, 0);
cv::Mat state(6, 1, CV_32F);
cv::Mat meas(3, 1, CV_32F);
bool initialized = false;

void initializeKalmanFilter() {
    kf.transitionMatrix = (cv::Mat_<float>(6, 6) << 
                            1, 0, 0, 1, 0, 0,
                            0, 1, 0, 0, 1, 0,
                            0, 0, 1, 0, 0, 1,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1);
    setIdentity(kf.measurementMatrix);
    setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, cv::Scalar::all(.1));
}

void updateKalmanFilter(cv::Vec3d& translation_vector) {
    if (!initialized) {
        // Initialize the state
        state.at<float>(0) = translation_vector[0];
        state.at<float>(1) = translation_vector[1];
        state.at<float>(2) = translation_vector[2];
        kf.statePost = state;
        initialized = true;
    }

    // Predict
    kf.predict();

    // Measurement
    meas.at<float>(0) = translation_vector[0];
    meas.at<float>(1) = translation_vector[1];
    meas.at<float>(2) = translation_vector[2];

    // Correct
    kf.correct(meas);

    // Update the translation vector with the filtered values
    translation_vector[0] = kf.statePost.at<float>(0);
    translation_vector[1] = kf.statePost.at<float>(1);
    translation_vector[2] = kf.statePost.at<float>(2);
}

/////////////////////////////////////////////////////////////////////////////////////////
void configureDetectorParameters() {
    detector_params->adaptiveThreshWinSizeMin = 3;
    detector_params->adaptiveThreshWinSizeMax = 23;
    detector_params->adaptiveThreshWinSizeStep = 10;
    detector_params->minMarkerPerimeterRate = 0.03;
    detector_params->maxMarkerPerimeterRate = 2;
    detector_params->polygonalApproxAccuracyRate = 0.03;
    detector_params->minCornerDistanceRate = 0.05;
    detector_params->minDistanceToBorder = 1;
    detector_params->minMarkerDistanceRate = 0.05;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void int_handler(int x) {
    // disconnect and exit gracefully
    if (show_detections) {
        cv::destroyAllWindows();
    }
    ros::shutdown();
    exit(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec) {
    	
	
	return {vec[0], vec[1], vec[2]};
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

double getPrec(std::vector<int> ids, int i){
 	vector<int> current_vector(num_detected);
	current_vector = ids_hashmap[ids[i]];
    int num_detections = std::accumulate(current_vector.begin(), current_vector.end(), 0); // vector element sum
	return (double) num_detections/num_detected *100;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector) {
    Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

tf2::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector) {
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
    return transform;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void callback_camera_info(const CameraInfoConstPtr &msg) { 
    //CameraInfoConstPtr -> sensor_msg.h group
    //https://docs.ros.org/en/diamondback/api/sensor_msgs/html/CameraInfo_8h.html

    // if (camera_model_computed) {
    //     return;
    // }
    // camera_model.fromCameraInfo(msg); //return fromCameraInfo(*msg) -> boolian
    // camera_model.distortionCoeffs().copyTo(distortion_coefficients); // cv::Mat_<double> D_; // Unaffected by binning, ROI
    // intrinsic_matrix = camera_model.intrinsicMatrix(); // cv::Matx33d K_; // Describe current image (includes binning, ROI)
    // camera_model_computed = true; 
    // ROS_INFO("camera model is computed");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void update_params_cb(const std_msgs::Empty &msg)
{
	// update the parameters:
	//nh.getParam("/blur_window_size", blur_window_size);
} 

std_msgs::Float32MultiArray return_trans_vec(Vec3d vector){
    
    std_msgs::Float32MultiArray result;
    result.data.resize(3);
    float dumi_0 =  static_cast<float>(vector[0]);
    float dumi_1 = vector[1];
    float dumi_2 = vector[2];

    result.data[0] = vector[0];
    result.data[1] = vector[1];
    result.data[2] = vector[2];



    return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void callback(const cv::Mat &image, cv::Mat intrinsic_matrix, cv::Mat distortion_coefficients) {
    
    // if (!camera_model_computed) {
    //     ROS_INFO("camera model is not computed yet");
    //     return;
    // }

    string frame_id = "d435i_frame"; //image_msg->header.frame_id;
    //cv::Mat image = color;//cv_bridge::toCvShare(image_msg)->image; // matrix data -> image data extract
    

    
    imshow("INITIAL IMAGE",image); // show extracted image

    // Smooth the image to improve detection results
    // if (enable_blur) {
    //     GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0,
    //                  0); // blur를 허용했을때 허용한 만큼 보정하는 텀
    // }
    
    // Detect the markers
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected; // 2차원 배열
    
    aruco::detectMarkers(image, dictionary, corners, ids, detector_params, rejected); // 뜯어볼 수 없음 ㅠ.ㅠ
 
    // publish aruco info:  
    alfons_msgs::ArucoInfo ar_msg;
    
    
    for(int i = 0;i<ids.size();i++)
    {
        vector<Point2f> one_corner = corners[i];
        ar_msg.marker_ids.push_back(ids[i]);
        ar_msg.center_x_px.push_back((one_corner[0].x+one_corner[1].x+one_corner[2].x+one_corner[3].x)/4);
        ar_msg.center_y_px.push_back((one_corner[0].y+one_corner[1].y+one_corner[2].y+one_corner[3].y)/4);
    }

    ar_msg.header.stamp = ros::Time::now();
    ar_msg.header.frame_id = "camera";
    
    aruco_info_pub_.publish(ar_msg);

    // Show image if no markers are detected
    if (ids.empty()) {
       // ROS_INFO("Markers not found");
       cv::putText(image, "no markers found", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
        if (show_detections) {
            //imshow("markers", display_image);
	if (result_img_pub_.getNumSubscribers() > 0)
	{
        
		result_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
	}
            auto key = waitKey(1);
            if (key == 27) {
                ROS_INFO("ESC pressed, exit the program");
                ros::shutdown();
            }
        }
       // return;
    }

if(ids.size()>0)
{
   
    // Compute poses of markers
    vector<Vec3d> rotation_vectors, translation_vectors;
    aruco::estimatePoseSingleMarkers(corners, marker2_size, intrinsic_matrix, distortion_coefficients,
                                     rotation_vectors, translation_vectors);

    // Draw marker poses
   
    for(int i=0; i<ids.size(); i++){
        //updateKalmanFilter(rotation_vectors[i]);
        //solvePnP(objPoints, corners[i], intrinsic_matrix, distortion_coefficients, rotation_vectors, translation_vectors);
        aruco::drawDetectedMarkers(image, corners, ids);
        //ROS_INFO("--------------[%d]-----------------",ids[i]);
        //ROS_INFO_STREAM(rotation_vectors[0]);
        //ROS_INFO("-------------------------------");
    cv::drawFrameAxes(image, intrinsic_matrix, distortion_coefficients, rotation_vectors[i], translation_vectors[i], 0.1);
    // ROS_INFO("-------------------------");
    }
    //ROS_INFO_STREAM(corners.size());

	if (result_img_pub_.getNumSubscribers() > 0)
	{

                cv::putText(image, ""+SSTR(image_width)+"x"+SSTR(image_height)+"@"+SSTR(image_fps)+"FPS m. size: "+SSTR(marker_size)+" m"+" blur: "+SSTR(blur_window_size), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 255, 0), 2);

	for(int i = 0; i<ids.size();i++)
	{
	double prec = getPrec(ids,i); // precentage
	if(prec>=min_prec_value) // precentage
        {
        Vec3d distance_z_first = translation_vectors[i];
        double distance_z = ROUND3(distance_z_first[2]);
        cv::putText(image, "id: "+SSTR(ids[i])+" z dis: "+SSTR(distance_z)+" m  "+SSTR(ROUND2(prec))+" %", cv::Point(10, 70+i*30), cv::FONT_HERSHEY_SIMPLEX, 0.9, CV_RGB(0, 255, 0), 2);
        result_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
        }
	}
	}	
        imshow("output", image); // opencv im_show
        auto key = waitKey(1);
        if (key == 27) {
            ROS_INFO("ESC pressed, exit the program");
            ros::shutdown();
        }


    // Publish TFs for each of the markers
    static tf2_ros::TransformBroadcaster br;
    auto stamp = ros::Time::now();

    // Create and publish tf message for each marker
    tf2_msgs::TFMessage tf_msg_list;
    for (auto i = 0; i < rotation_vectors.size(); ++i)
    {

	    if(getPrec(ids,i)>min_prec_value)
        {
            //ROS_INFO("aruco markers tf");
            auto translation_vector = translation_vectors[i];
            auto rotation_vector = rotation_vectors[i];
            auto transform = create_transform(translation_vector, rotation_vector);
            //
            //trans_Vec_pub_.publish(return_trans_vec(translation_vectors[0]));
            //
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = frame_id;
            stringstream ss;
            ss << marker_tf_prefix << ids[i];
            tf_msg.child_frame_id = ss.str();
            tf_msg.transform.translation.x = transform.getOrigin().getX();
            tf_msg.transform.translation.y = transform.getOrigin().getY();
            tf_msg.transform.translation.z = transform.getOrigin().getZ();
            tf_msg.transform.rotation.x = transform.getRotation().getX();
            tf_msg.transform.rotation.y = transform.getRotation().getY();
            tf_msg.transform.rotation.z = transform.getRotation().getZ();
            tf_msg.transform.rotation.w = transform.getRotation().getW();
            tf_msg_list.transforms.push_back(tf_msg);
            br.sendTransform(tf_msg);
        }
    }
    tf_list_pub_.publish(tf_msg_list); 

}

// rotate vector:
// [ 1 2 3 4 5 ]  --> [ 5 1 2 3 4 ]
if(num_detected>0)
{
//std::cout<<"ids size: vor erase"<<ids.size()<<std::endl;
map<int, vector<int>>::iterator il;
for ( il = ids_hashmap.begin(); il != ids_hashmap.end(); il++ )
{  
 vector<int> current_vector(num_detected);
  current_vector = il->second;
 rotate(current_vector.begin(),current_vector.end()-1,current_vector.end());
il->second = current_vector;
}

// gehe alle in der Liste bestehenden durch:
map<int, vector<int>>::iterator it;
for ( it = ids_hashmap.begin(); it != ids_hashmap.end(); it++ )
{  
  bool current_id_was_found = false;
  for(int j=0;j<ids.size();j++)
  {
   if( (ids[j] == it->first) && (it->second.size()>1))
   {
	current_id_was_found = true;
        ids.erase (ids.begin()+j);
	//std::cout<<"erase "<<ids[j]<<"it first"<<it->first<<"size_second "<<it->second.size()<<std::endl;
   }
  }
  vector<int> current_vector(num_detected);
  current_vector = it->second;
  current_vector[0] = 0;
  if (current_id_was_found)
  {
   current_vector[0] =1;
  //std::cout<<" 1 was set"<<it->first<<std::endl;
  }
it->second = current_vector;
}


// adde alle restlichen in ids (das sind die neu erkannten)
for(int i = 0;i<ids.size();i++)
{

   std::map<int, vector<int>>::iterator ittt = ids_hashmap.begin();
   vector<int> tmpp(num_detected, 0);
   tmpp[0] = 1;
   std::string aa = "";
   for(int i=0;i<num_detected;i++)
	aa+=SSTR(tmpp[i])+",";

   ids_hashmap.insert(make_pair(ids[i], tmpp));
   //std::cout<<"added new: "<<ids[i]<<" "<<aa<<" size tmpp"<<tmpp.size()<<std::endl;
}

//// print the hashmap:
map<int, vector<int>>::iterator itt;
for ( itt = ids_hashmap.begin(); itt != ids_hashmap.end(); itt++ )
{
   vector<int> tmp(num_detected, 0);
   tmp = itt->second; 
// hack -> no idea why this is necessary
if(itt->second.size() ==0)
{
   vector<int> tmpe(num_detected, 0);
   tmpe[0] = 1;
itt->second =tmpe;
} // end of hack
//   std::string a = SSTR(itt->first)+"  "+SSTR(itt->second.size())+ " ";
	


//   for(int i=0;i<tmp.size();i++)
//{
//   a+= SSTR(tmp[i])+",";
//}
//std::cout<<a<<std::endl;
}
} // num_detected>0

}

//////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;

  tf_list_pub_    = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 10);
  aruco_info_pub_ = nh.advertise<alfons_msgs::ArucoInfo>("/aruco_list", 10); 
  trans_Vec_pub_  = nh.advertise<std_msgs::Float32MultiArray>("/trans_vec_data",1);
  //------------------------------------------------------------------------//

  initializeKalmanFilter();

  /* Contruct a pipeline which abstracts the device */
  rs2::pipeline pipe;
  //
  //rs2::pipeline pipe_ir;

  /* Create a configuration for configuring the pipeline with a non default profile */
  rs2::config cfg;
  //
  //rs2::config cfg_ir;
  
  /* Camera warmup - dropping several first frames to let auto-exposure stabilize */
  rs2::frameset frames;
  //
  //rs2::frameset ir_frames;

  /* Get each frame */
  rs2::frame color_frame;
  //
  //rs2::frame color_ir_frame;

  /* Add desired streams to configuration */
  std::string serial_num_d451i = "134322070885"; // d435i serial number
  std::string serial_num_t265 = "126622110919"; // t265 serial number

  cfg.enable_device(serial_num_d451i);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  //
  //cfg_ir.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);

  /* Instruct pipeline to start streaming with the requested configuration */
  pipe.start(cfg);
  //
  //pipe_ir.start(cfg_ir);
  //////////////////////////////////////////////////////////////////////////////

  char buf[256];           // 이미지 경로를 저장하기 위해 선언
	int index = 0;           // 이미지 index화

  std::vector<cv::Point> ir1_arr;  // findChessboardCorners를 위해 선언하였지만 값은 사용하지 않는 dummy value
  
  
  
  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }


    cv::CommandLineParser parser(argc, argv, fdcl::keys);



    int dictionary_id = parser.get<int>("d");
    int wait_time = 10;


 ///////////////////////////////////////////////////
  


  //namedWindow("Display Imagee", WINDOW_AUTOSIZE);

  ros::Rate loop_rate(200);


// float arr_cam_mat[9] = {6.2778611466799782e+02, 0., 3.7393052963985235e+02, 0.,
//        6.2123669424614070e+02, 2.6210887780634675e+02, 0., 0., 1. };

float arr_cam_mat[9] = {617.306, 0, 320.0,
0, 617.306, 240.0,
0, 0, 1};

// float arr_dist_coe[5] = {  2.9159262790263624e-01, -9.9290536008974140e-01,
//        1.2972766825910515e-02, 5.3373833442882039e-02,
//        3.0216260830627721e+00};
float arr_dist_coe[5] = {  0,0,0,0,0};
  cv::Mat cameraMatrix(3,3,CV_32F,arr_cam_mat);

  cv::Mat distCoeffs(1,5,CV_32F,arr_dist_coe);
    //cv::Mat cameraMatrix, distCoeffs;
  
  if (!cv::ocl::haveOpenCL()) {
        std::cerr << "OpenCL is not available on this system." << std::endl;
        return -1;
    }

  // OpenCL 사용 설정
    cv::ocl::setUseOpenCL(true);
    if (!cv::ocl::useOpenCL()) {
        std::cerr << "Failed to enable OpenCL." << std::endl;
        return -1;
    }

    std::cout << "OpenCL is enabled." << std::endl;

  // Process the video
    while (ros::ok()) {
        cv::Mat image, image_copy;

        frames = pipe.wait_for_frames();
        color_frame = frames.get_color_frame();
    
        cv::Mat color(Size(640,480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
	
	// 이미지 크기 조정
    //cv::resize(color, color, cv::Size(color.cols / 2, color.rows / 2));

    int width = color.cols;
    int height = color.rows;
    size_t imgSize = width * height * color.channels();
    size_t grayImgSize = width * height;

    // OpenCL 초기화
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    if (platforms.empty()) {
        std::cerr << "No OpenCL platforms found." << std::endl;
        return -1;
    }
    cl::Platform platform = platforms[0];

    std::vector<cl::Device> devices;
    platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
    if (devices.empty()) {
        std::cerr << "No OpenCL devices found." << std::endl;
        return -1;
    }
    cl::Device device = devices[0];

    cl::Context context(device);
    cl::Program::Sources sources;
    sources.push_back({kernelSource, strlen(kernelSource)});

    cl::Program program(context, sources);
    if (program.build({device}) != CL_SUCCESS) {
        std::cerr << "Error building OpenCL program: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;
        return -1;
    }

    cl::Buffer inputBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, imgSize, color.data);
    cl::Buffer outputBuffer(context, CL_MEM_WRITE_ONLY, grayImgSize);

    cl::Kernel kernel(program, "rgb_to_gray");
    kernel.setArg(0, inputBuffer);
    kernel.setArg(1, outputBuffer);
    kernel.setArg(2, width);
    kernel.setArg(3, height);

    cl::CommandQueue queue(context, device);
    queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(width, height));
    queue.finish();

    std::vector<unsigned char> grayImgData(grayImgSize);
    queue.enqueueReadBuffer(outputBuffer, CL_TRUE, 0, grayImgSize, grayImgData.data());

    cv::Mat grayImg(height, width, CV_8UC1, grayImgData.data());

	
        
	
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        
        detector_params = cv::aruco::DetectorParameters::create();
        configureDetectorParameters();

        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); //DICT_4X4_1000 이 옵션이 성능 결정
	
	
	callback(grayImg, cameraMatrix, distCoeffs);
        //callback(color,cameraMatrix,distCoeffs);


        // cv::aruco::detectMarkers(color, dictionary, corners, ids, detector_params, rejectedCandidates);


        
        // cv::aruco::estimatePoseSingleMarkers(corners, 0.0218f, cameraMatrix, distCoeffs, rvecs, tvecs);
        
        // //cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);

        // cv::Mat outputImage = color.clone();
        // if (ids.size() > 0) {
        //     cv::aruco::drawDetectedMarkers(outputImage, corners, ids);
        // }

        //  for(int i=0; i<ids.size(); i++){
        // cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        // // ROS_INFO("-----------------------------");
        // ROS_INFO_STREAM(tvecs[0].val[2]);
        // ROS_INFO("-----------------------------");
        // }


        // Publisher:
        // image_transport::ImageTransport it(nh);       
        // result_img_pub_ = it.advertise("/result_img", 1);
    


        //imshow("Detected markers", outputImage);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27) {
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }


  return 0;
}

