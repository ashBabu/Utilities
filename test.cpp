#include <iostream>
#include <ctime>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>

#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <boost/circular_buffer.hpp>
#include "harvester_trajectory_planning/Data.h"
#include "harvester_trajectory_planning/decision_strategy/Decision.h"
#include <test_localization/Perception.h>
#include <chrono>
#include <thread>
#include <algorithm>

//    https://www.boost.org/doc/libs/1_39_0/libs/statechart/doc/tutorial.html


#include <test_localization/Perception.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Perception
{


Perception(ros::NodeHandle& _nh)
{
    typedef std::shared_ptr<Perception> Ptr;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::cout<< "Waiting for lookupTransform()..." << std::endl;
    while (true) {
        try {
            geometry_msgs::TransformStamped r_T_c_ = tfBuffer.lookupTransform("base_link", "perception_link",
                                              ros::Time(0));
//            ros::Duration(1.0).sleep();
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
        break;
    }
    std::cout << "... done! :)" << std::endl;

    ros::Subscriber sub_ = _nh.subscribe("/robot_perception/results", 0, Callback, this);       ///  Raspberry Perception
}

~Perception()
{
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Callback(const rasberry_perception::DetectionsConstPtr &berries)
{
    sb_.clear();
    rasberry_perception::Detections detections(*berries);


    for (const auto& berry : berries->objects)
    {
        const int ripe_or_raw = berry.class_name=="unripe" ? 0 : 1;


        geometry_msgs::Pose pose;
        pose.position.x = berry.pose.position.x;
        pose.position.y = berry.pose.position.y;
        pose.position.z = berry.pose.position.z + berry.size.z/4; // z position - depth so it in the center of the berry (inside)
        /// conversion from camera base to arm base
        transformPose(pose);

        Strawberry s{};
        s.ripe = ripe_or_raw;
        s.id = berry.track_id;
        s.x = pose.position.x;
        s.y = pose.position.y;
        s.z = pose.position.z;
        s.dim_x = berry.size.x;
//        s.dim_y = berry.size.z; // swap dim w.r.t. base_link
        s.dim_y = berry.size.y; // swap dim w.r.t. base_link
//        s.dim_z = berry.size.y; // swap dim w.r.t. base_link
        s.dim_z = berry.size.z; // swap dim w.r.t. base_link
        sb_.push_back(s);
    }

}


void transformPose(geometry_msgs::Pose &out)
{
    tf2::doTransform(out, out, r_T_c_);
}

std::vector<Strawberry> getList()
{
    return sb_;
}

};





//// START OF STATE MACHINE


namespace sc = boost::statechart;

struct MyStruct
{
    double x;
    double y;
    double z;
    double dim_x;
    double dim_y;
    double dim_z;
    bool re;
    unsigned int id;
};


template<typename T>
void getOptimal(const T &sb, std::vector<MyStruct> &rb)
{}


struct stop : sc::event< stop > {};
struct move : sc::event< move > {};

struct Active;

struct Stopped;

struct MyList
{
    virtual std::vector<MyStruct> getList() const = 0;
};

struct StateMachine : sc::state_machine< StateMachine, Active >
{
public:
    std::vector<MyStruct> getList() const
    {
        return state_cast< const MyList & >().getList();
    }
    StateMachine(ros::NodeHandle &nh)
    {
        perception1 = std::make_shared<perception::Perception>(nh);
    }
    perception::Perception::Ptr perception1;
};

struct Active : sc::simple_state< Active, StateMachine, Stopped > {
public:
    typedef sc::transition<move, Active> reactions;

    Active() = default;
    std::vector<MyStruct> getList() {
        StateMachine sm(ros::NodeHandle nh);

        bb = sm.perception1->getList();
        for (auto &b: bb)
            MyStructCircBuff_.push_back(b);

        getOptimal(MyStructCircBuff_, rbb);
        return rbb;
    }

private:

    std::vector<MyStruct> bb, rbb;
    boost::circular_buffer<MyStruct> MyStructCircBuff_{20};
};

struct Moving : MyList,  sc::simple_state< Moving, Active >
{
public:
    typedef sc::transition< stop, Stopped > reactions;

    Moving() = default;

    ~Moving()
    {
        context< Active >().getList() ;
    }
};

struct Stopped : MyList,  sc::simple_state< Stopped, Active >
{
    typedef sc::transition< move, Moving > reactions;

    Stopped() = default;

    virtual std::vector<MyStruct> getList()
    {
        return context< Active >().getList();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ThreeD_localizer");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    StateMachine myWatch(nh);

    return 0;
}