#include "main.h"

#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exit" << std::endl;
  exit(1);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "tsdf_pcd_node");
    ros::NodeHandle nh("~");

    tsdf_pcd_class tsdf_pcd_(nh);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}