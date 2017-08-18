#include <iostream>
#include <ros/ros.h>
#include "MapGenerator.h"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalmap_generator");
    ros::start();

    ros::NodeHandle node;
    MapGenerator mapgen(node);
    
    
    ros::Rate r(30);
    while (ros::ok())
    {
        mapgen.Refresh();
        ros::spinOnce();
        r.sleep();
    }
    ros::shutdown();
    return 0;
}
