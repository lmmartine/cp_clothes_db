#include <cp_clothes_db.h>

namespace uchile_perception {

CPCLOTHESDB::CPCLOTHESDB(string name):_name(name),_enabled(false) {

    ros::NodeHandle priv("~"), nh;
    // - - - - - - - - l i s t e n e r s - - - - - - - - - - - -
    _tf_listener = new tf::TransformListener(ros::Duration(10.0));

    // - - - - - - - p a r a m e t e r s - - - - - - - - - - -

    priv.param<std::string>("base_frame",_base_frame,"base_link");
    priv.param<std::string>("rgbd_frame",_rgbd_frame,"xtion2_link");
    priv.param<std::string>("rgb_frame",_rgb_frame,"xtion2_rgb_optical_frame");
    priv.param<std::string>("depth_frame",_depth_frame,"xtion2_depth_optical_frame");
    priv.param<std::string>("point_topic",_point_topic,"/xtion2/depth/points");
    priv.param<std::string>("rgb_topic",_rgb_topic,"/xtion2/rgb/image_raw");
    priv.param<std::string>("depth_topic",_depth_topic,"/xtion2/depth/image_raw");
    priv.param<std::string>("gripper_frame", _gripper_frame, "/r2_ee");
    priv.param<std::string>("class", _class, "pant");
    priv.param<int>("id_move", _idmove, 1);
    priv.param<int>("nxtion", nxtion, 1);
    priv.param<bool>("cut_robot", _cut_robot, false);
    priv.param<float>("crop_width", _crop_width, 2.0);
    priv.param<float>("crop_depth", _crop_depth, 2.0);
    priv.param<float>("crop_min_z", _crop_min_z, 0.75);
    priv.param<float>("crop_max_z", _crop_max_z, 2.0);



    // variables
    last_gripper_position = 0;
    _idimg = 1;

    // nxtion = 1;
    // if (_depth_topic.find("xtion2") != std::string::npos)
    //     nxtion = 2;
     ROS_INFO_STREAM("Working with xtion"<<nxtion);
    // - - - - - - - - p u b l i s h e r s  - - - - - - - - - - - -
    std::stringstream topic;
    topic << "/xtion"<<nxtion<<"/mask/image_raw";

    image_transport::ImageTransport it(nh);
    _mask_pub = it.advertise(topic.str(), 1);

    // - - - - - - - - s e r v i c e s  - - - - - - - - - - - - -
    _active_server  = priv.advertiseService("active", &CPCLOTHESDB::_active_service, this);

}

CPCLOTHESDB::~CPCLOTHESDB() {
    delete _tf_listener;
}

 bool CPCLOTHESDB::_active_service(cp_clothes_db::Onoff::Request  &req, cp_clothes_db::Onoff::Response &res) {

    if(req.select == true) {
        if (!_is_on) {
            _subs_point = priv.subscribe(_point_topic, 1, &CPCLOTHESDB::_process_point, this); 
            _subs_depth = priv.subscribe(_depth_topic, 1, &CPCLOTHESDB::_process_depth, this); 
            _subs_rgb = priv.subscribe(_rgb_topic, 1, &CPCLOTHESDB::_process_rgb, this); 
            _is_on = true;

            ROS_INFO_STREAM("Turning on "+_imagePoint_topic+". . . OK");
        } else ROS_DEBUG_STREAM("Already turned on");
    }
    else{
        if (_is_on) {
              _subs_point.shutdown();
              _subs_depth.shutdown();
              _is_on = false;
              ready_point = false;
              ROS_INFO_STREAM(" Turning off . . . OK");
        } else  ROS_DEBUG_STREAM("Already turned off"); 
    }
    return true;
}

void CPCLOTHESDB::_process_point(const pcl::PCLPointCloud2::ConstPtr &point_cloud_in){

    if(!_is_on) return;
    cloud_in=point_cloud_in;
    ready_point = true;
}

 void CPCLOTHESDB::_process_rgb(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    rgb_in=img;
    // image_frameid = img->header.frame_id;
    ready_rgb = true;
}

 void CPCLOTHESDB::_process_depth(const sensor_msgs::ImageConstPtr& img){
    if(!_is_on) return;
    depth_in=img;
    image_frameid = img->header.frame_id;
    ready_depth = true;
}

void CPCLOTHESDB::crop_pointcloud(PointCloud cloud_in, PointCloud::Ptr cloud_out) {
    PointCloud::Ptr cloud_ptr  = cloud_in.makeShared();

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;

    // filter z
    pass.setInputCloud (cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (_crop_min_z, _crop_max_z);
    pass.filter (*cloud_out);

}

geometry_msgs::Pose CPCLOTHESDB::get_gripper_position (){
    tf::StampedTransform transform;
    geometry_msgs::Pose gripper;

    try{
      _tf_listener->lookupTransform(_base_frame, _gripper_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    lasttime_transform = transform.stamp_;
    gripper.position.x = transform.getOrigin().getX();
    gripper.position.y = transform.getOrigin().getY();
    gripper.position.z = transform.getOrigin().getZ();

    gripper.orientation.x = transform.getRotation().getX();
    gripper.orientation.y = transform.getRotation().getY();
    gripper.orientation.z = transform.getRotation().getZ();
    gripper.orientation.w = transform.getRotation().getW();

    return gripper;

}

void CPCLOTHESDB::crop_robot_pointcloud(PointCloud cloud_in, PointCloud::Ptr cloud_out, float gripper_position) {

    PointCloud::Ptr cloud_ptr  = cloud_in.makeShared();
    pcl::PassThrough<pcl::PointXYZ> pass;

    // filter z
    pass.setInputCloud (cloud_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (_crop_min_z, gripper_position);
    pass.filter (*cloud_out);

    // //     // x: depth
    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-_crop_depth, _crop_depth);
    pass.filter (*cloud_out);
    // y: width
    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-_crop_width, -0.5);
    pass.filter (*cloud_out);

}

void CPCLOTHESDB::save_images (cv::Mat rgbimg, cv::Mat  depthimg, cv::Mat mask){



    // Save opencv images 
    std::stringstream name_out, rgb_name_out, depth_name_out, mask_name_out;

    name_out<<ros::package::getPath("cp_clothes_db")<<"/IMG_OUT/"<<_class<<"/"<<_class;

    bool numdetected = false;

    rgb_name_out << name_out.str()<<_idmove<<"_x"<<nxtion<<"img"<<_idimg <<"_rgb.png";
    depth_name_out << name_out.str()<<_idmove<<"_x"<<nxtion<<"img"<<_idimg <<"_depth.png";
    mask_name_out << name_out.str()<<_idmove<<"_x"<<nxtion<<"img"<<_idimg <<"_mask.png";

    _idimg ++;
    cv::imwrite(rgb_name_out.str(), rgbimg);
    cv::imwrite(depth_name_out.str(), depthimg);
    cv::imwrite(mask_name_out.str(), mask);

    ROS_INFO_STREAM("[" << _name << "]  Saved images '"
                << name_out.str()<<_idmove<<"_x"<<nxtion<<"img"<<_idimg );
}



// - - - - -  RUN   - - - - - - - - - -
void CPCLOTHESDB::run() {

    float gripper_position = get_gripper_position().position.z;
    if (std::abs(last_gripper_position - gripper_position) < 0.005) return;    


    ready_point = false;
    ready_depth = false;
    
    cv::Mat ImageIn, DepthIn;
    ImageIn  = cv_bridge::toCvCopy((rgb_in), sensor_msgs::image_encodings::BGR8)->image;
    DepthIn = cv_bridge::toCvShare(depth_in)->image;

    // // Clouds
    PointCloud::Ptr cloud_helper      (new PointCloud);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_helper);
    PointCloud transformed_cloud, cloud_process; // transformed cloud
    PointCloud::Ptr filter_cloud (new PointCloud);

    // iterators
    PointCloud::iterator c_it;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step1: Transform cloud to '_base_frame'
  
    try {
        cloud_helper->header.frame_id = _depth_frame;
        bool success_transformation = pcl_ros::transformPointCloud(_base_frame, *cloud_helper, transformed_cloud, *_tf_listener);
        if (!success_transformation) return;

    } catch (tf::TransformException & ex) {
        ROS_ERROR_STREAM("[" << _name << "] Failed to transform rgbd pointcloud2 in '"
                << cloud_helper->header.frame_id << "' frame, to '" << _base_frame << "' : " << ex.what());
        return;
    }


    // Step2: Crop table, robot and walls
    if (_cut_robot)
        crop_robot_pointcloud(transformed_cloud, filter_cloud, gripper_position);
    else
        crop_pointcloud(transformed_cloud, filter_cloud);


     // Step3: Transform cloud to '_depth_frame'
    filter_cloud->header.frame_id = _base_frame;

    try {
        bool success_transformation = pcl_ros::transformPointCloud(_rgb_frame, *filter_cloud, cloud_process, *_tf_listener);
        if (!success_transformation) return;
    } catch (tf::TransformException & ex) {

        ROS_ERROR_STREAM("[" << _name << "] Failed to transform rgbd pointcloud2 in '"
                << filter_cloud->header.frame_id << "' frame, to '" << _depth_frame << "' : " << ex.what());
        return;
    }


    // Stet4: Get Mask
    int mask_W = 640, mask_H=480;
    float cx=319.5,  cy = 239.5; //rgb intrinsic
    float fx = 525.0, fy = 525.0;
    //float cx=314.5,  cy = 235.5; //depth intrinsic
    //float fx = 570.3422241210938, fy = 570.3422241210938;
    float kfactor= 0.0019047619;
    float scalingFactor = 5000.0;

    cv::Mat mask =  cv::Mat::zeros(mask_H, mask_W, CV_8UC1);

    for ( c_it = cloud_process.begin(); c_it < cloud_process.end(); c_it++) {

        if (std::isfinite(c_it->x) && std::isfinite(c_it->y) && std::isfinite(c_it->z) )
        {
            int x_2d,y_2d;
            float x_3d = c_it->x, y_3d = c_it->y, z_3d = c_it->z;

            x_2d = x_3d*fx/z_3d  + cx;
            y_2d = y_3d*fy/z_3d  + cy;

            if (x_2d>=0 && y_2d>=0 && x_2d<mask_W && y_2d<mask_H)
                mask.at<uchar>(y_2d,x_2d) = (uchar)255;

        }

    }


    // convert OpenCV image to ROS message DepthIn
    cv_bridge::CvImage cvi;
    cvi.header.stamp =  ros::Time::now();
    cvi.header.frame_id = "image";
    cvi.encoding = "mono8";
    cvi.image = mask;

    sensor_msgs::Image  msg_mask;
    cvi.toImageMsg(msg_mask);

     if (_mask_pub.getNumSubscribers() > 0) {
            _mask_pub.publish(msg_mask);
    }

    save_images (ImageIn,  DepthIn, mask);
    last_gripper_position = gripper_position;
}




} /* namespace uchile_perception */

int main(int argc, char** argv) {

    ros::init(argc, argv, "cp_clothes_db");
    ros::NodeHandle priv("~");

    boost::scoped_ptr<uchile_perception::CPCLOTHESDB> node(
            new uchile_perception::CPCLOTHESDB(ros::this_node::getName())
    );


    int _fps = 25;
    if(!priv.getParam("fps",_fps)) priv.setParam("fps",_fps);

    ros::Rate r(_fps);

    while(ros::ok()){
        if (node->_is_on && node->ready_point && node->ready_depth && node->ready_rgb)
          node->run();
          
        // r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Quitting ... \n");

    return 0;
}
