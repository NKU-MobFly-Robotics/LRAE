#include "visualization_msgs/Marker.h"
#include "string.h"
#include "ros/ros.h"
namespace visual
{
    class Marker
    {
    private:
        std::string pub_topic_;
        std::string frame_id_;
        ros::Publisher marker_pub_;

    public:
        // static int id_;
        visualization_msgs::Marker marker_;

        explicit Marker(ros::NodeHandle* nh, std::string pub_topic, std::string frame_id)
            : pub_topic_(pub_topic), frame_id_(frame_id)
        {
            marker_pub_ = nh->advertise<visualization_msgs::Marker>(pub_topic_, 2);
            // id_++;
            marker_.id = 0;
            marker_.action = visualization_msgs::Marker::ADD;
            marker_.pose.orientation.w = 1.0;
            marker_.header.frame_id = frame_id_;
        }
        explicit Marker(ros::NodeHandle& nh, std::string pub_topic, std::string frame_id)
            : pub_topic_(pub_topic), frame_id_(frame_id)
        {
            marker_pub_ = nh.advertise<visualization_msgs::Marker>(pub_topic_, 2);
            // id_++;
            marker_.id = 0;
            marker_.action = visualization_msgs::Marker::ADD;
            marker_.pose.orientation.w = 1.0;
            marker_.header.frame_id = frame_id_;
        }

        ~Marker() = default;

        void SetColorRGBA(const std_msgs::ColorRGBA& color)
        {
            marker_.color = color;
        }
        void SetColorRGBA(double r, double g, double b, double a)
        {
            marker_.color.r = r;
            marker_.color.g = g;
            marker_.color.b = b;
            marker_.color.a = a;
        }
        void SetScale(double x, double y, double z)
        {
            marker_.scale.x = x;
            marker_.scale.y = y;
            marker_.scale.z = z;
        }
        void SetType(int type)
        {
            marker_.type = type;
        }
        void SetAction(visualization_msgs::Marker::_action_type action)
        {
            marker_.action = action;
        }
        void Publish()
        {
            marker_pub_.publish(marker_);
        }
        void PushBackWinLine(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, geometry_msgs::Point p4)
        {
            marker_.points.push_back(p1);
            marker_.points.push_back(p2);
            marker_.points.push_back(p2);
            marker_.points.push_back(p3);
            marker_.points.push_back(p3);
            marker_.points.push_back(p4);
            marker_.points.push_back(p4);
            marker_.points.push_back(p1);
        }
        void PushBackLine(geometry_msgs::Point p1, geometry_msgs::Point p2)
        {
            marker_.points.push_back(p1);
            marker_.points.push_back(p2);
        }
        typedef std::shared_ptr<Marker> Ptr;
    };

}