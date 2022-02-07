#ifndef _dynamic_transform_publisher_H
#define _dynamic_transform_publisher_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#define DEG2RAD 0.0174532925
#define RAD2DEG 57.2958


class DynamicTransformPublisher{
    public:
        DynamicTransformPublisher();
        ~DynamicTransformPublisher();
    private:
        ros::NodeHandle nh_, pnh_;

        ros::Timer tf_timer_;

        ///////////////////////////
        //ROSParams
        ///////////////////////////
        std::string camera_name_; //camera name that will be prepended to topics subscribed

        //Rotation method
        int rotation_method_;

        //Translation
        double trans_x_, trans_y_, trans_z_;
        //Quaternion Orientation
        double quat_x_, quat_y_, quat_z_, quat_w_;

        //Rotation about the axes in radians
        double rot_x_rad_, rot_y_rad_, rot_z_rad_;
        //Rotation about the axes in degrees
        double rot_x_deg_, rot_y_deg_, rot_z_deg_;
        
        std::string child_frame_, parent_frame_;
        double publish_frequency_;

        ///////////////////////////
        //Publishers, Subscribers and Services
        ///////////////////////////
        

        ///////////////////////////
        //Ddynamic reconfigure
        ///////////////////////////
        std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;


    private:

        /**
         * @brief Read params from ROSParam server
         * and initialize other internal parameters
         */
        void initParams();

        /**
         * @brief Initialize publishers, subscribers and services
         */
        void initPubSubSrv();

        /////////////////////
        //Callbacks
        /////////////////////

        //Timer callback to publish Transform Timer 
        void publishTransformTimerCB(const ros::TimerEvent& event);

        /////////////////////
        //Helper methods
        /////////////////////


        // /**
        //  * @brief Convert from geometry_msgs::Pose to geometry_msgs::TransformStamped
        //  * 
        //  * @param pose geometry_msgs::Pose message
        //  * @param tf geometry_msgs::TransformStamped message
        //  */
        // void poseToTFMsg(const geometry_msgs::Pose& pose_msg, 
        //                 geometry_msgs::TransformStamped& tf_msg,
        //                 const std::string& parent_frame,
        //                 const std::string& child_frame);

        // /**
        //  * @brief Convert from geometry_msgs::Pose to tf2::Transform
        //  * 
        //  * @param pose_msg 
        //  * @param tf_transform 
        //  */
        // void poseToTFTransform(const geometry_msgs::Pose& pose_msg, 
        //                         tf2::Transform& tf_transform);


};

#endif //_dynamic_transform_publisher_H
