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
#include <test_localization/Perception.h>
#include <thread>
#include <algorithm>

//    https://www.boost.org/doc/libs/1_39_0/libs/statechart/doc/tutorial.html


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


struct Velocity{
public:
    double x, y, z;
    double getVelMagnitude() const
    {
        return sqrt((x*x + y*y + z*z));
    }
};


template<typename T>
void getOptimal(const T &sb, std::vector<MyStruct> &rb)
{}


void getMean(const std::vector<MyStruct> &strawberries, MyStruct & berry)
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