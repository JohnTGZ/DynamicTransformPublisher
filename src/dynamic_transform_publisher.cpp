#include "dynamic_transform_publisher/dynamic_transform_publisher.h"


DynamicTransformPublisher::DynamicTransformPublisher(): nh_(),  pnh_("~"){
    initParams();
    // initPubSubSrv();

    //Timers
    ros::Timer tf_timer = pnh_.createTimer(ros::Duration(1/publish_frequency_), &DynamicTransformPublisher::publishTransformTimerCB, this);
}

DynamicTransformPublisher::~DynamicTransformPublisher(){
    ROS_INFO("DynamicTransformPublisher destructed");
}


void DynamicTransformPublisher::initParams(){
    pnh_.param<std::string>("child_frame", child_frame_, "");
    pnh_.param<std::string>("parent_frame", parent_frame_, "");

    if (child_frame_ == "" || parent_frame_ == ""){
        ROS_ERROR("Please define child and parent frame!");
        return;
    }
    pnh_.param<double>("publish_frequency", publish_frequency_, 30);

    pnh_.param<int>("rotation_method", rotation_method_, 1);

    std::vector<double> translation, quat_orientation, rot_orientation;
    pnh_.getParam("translation", translation);
    pnh_.getParam("quat_orientation", quat_orientation);
    pnh_.getParam("rot_orientation", rot_orientation);

    trans_x_ = translation[0], trans_y_ = translation[1], trans_z_ = translation[2];

    quat_x_ = quat_orientation[0];
    quat_y_ = quat_orientation[1];
    quat_z_ = quat_orientation[2];
    quat_w_ = quat_orientation[3];

    rot_x_deg_ = rot_orientation[0];
    rot_y_deg_ = rot_orientation[1];
    rot_z_deg_ = rot_orientation[2];

    //Dynamic Reconfigure
    ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(pnh_));
    ddr_->registerVariable<int>("rotation_method", &rotation_method_, "rotation_method 1, 2, 3 and 4", 1, 5);
    
    ddr_->registerVariable<double>("Translation_X", &trans_x_, "X translation of origin", -100.0, 100.0);
    ddr_->registerVariable<double>("Translation_Z", &trans_z_, "Y translation of origin", -100.0, 100.0);
    ddr_->registerVariable<double>("Translation_Y", &trans_y_, "Z translation of origin", -100.0, 100.0);

    ddr_->registerVariable<double>("Quat_X", &quat_x_, "Rot. Mode 1: X quaternion", -1.0, 1.0);
    ddr_->registerVariable<double>("Quat_Y", &quat_y_, "Rot. Mode 1: Y quaternion", -1.0, 1.0);
    ddr_->registerVariable<double>("Quat_Z", &quat_z_, "Rot. Mode 1: Z quaternion", -1.0, 1.0);
    ddr_->registerVariable<double>("Quat_W", &quat_w_, "Rot. Mode 1: W quaternion", -1.0, 1.0);

    ddr_->registerVariable<double>("Roll", &rot_x_deg_, "Rot. Mode 2/3: Roll/X-axis", -360.0, 360.0);
    ddr_->registerVariable<double>("Pitch", &rot_y_deg_, "Rot. Mode 2/3: Pitch/Y-axis", -360.0, 360.0);
    ddr_->registerVariable<double>("Yaw", &rot_z_deg_, "Rot. Mode 2/3: Yaw/Z-axis", -360.0, 360.0);

    ddr_->publishServicesTopics();
}

void DynamicTransformPublisher::initPubSubSrv(){
}

void DynamicTransformPublisher::publishTransformTimerCB(const ros::TimerEvent& event){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    //Translation (x,y,z)
    transform.setOrigin( tf::Vector3(trans_x_, trans_y_, trans_z_) );
    //convert to radians
    rot_x_rad_ = DEG2RAD * rot_x_deg_;
    rot_y_rad_ = DEG2RAD * rot_y_deg_;
    rot_z_rad_ = DEG2RAD * rot_z_deg_;

    switch (rotation_method_){
        case 1:
            //Quaternions
            q.setValue(quat_x_, quat_y_, quat_z_, quat_w_);
            break;
        case 2:
            //Euler XYZ
            q.setEuler(rot_y_rad_,  rot_x_rad_, rot_z_rad_);
            break;
        case 3:
            //Euler ZYX
            q.setEulerZYX(rot_z_rad_, rot_y_rad_, rot_x_rad_);
            break;
        case 4:
            // RPY: Roll Pitch Yaw
            q.setRPY(rot_x_rad_, rot_y_rad_, rot_z_rad_);
            break;
        case 5: 
            //Axis angle notation
            // q.setRotation( );
            break;
        default:  
            tf::Quaternion q(quat_x_, quat_y_, quat_z_, quat_w_);
            break;
    }
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, child_frame_));

    // ROS_INFO_THROTTLE(2.0, "Roll(%f), Pitch(%f), Yaw(%f)", \
    //                 _rot_x_deg, _rot_y_deg, _rot_z_deg);
    // ROS_INFO_THROTTLE(2.0, "q_x(%f), q_y(%f), q_z(%f), q_w(%f)", \
    //                 q.getX(), q.getY(), q.getZ(), q.getW());
}
