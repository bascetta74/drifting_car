#include "serial_comm/serial_comm.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_comm");

    serial_comm serial_comm_node;

    serial_comm_node.Prepare();

    serial_comm_node.RunPeriodically();

    serial_comm_node.Shutdown();

    return (0);
}

