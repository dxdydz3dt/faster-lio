//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"

/// run the lidar mapping in online mode

DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    double total_time_for_savetrajectory;
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "faster_lio");
    ros::NodeHandle nh;

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS(nh);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    // Record the start time for the entire process
    auto start_time_total = std::chrono::high_resolution_clock::now();

    // online, almost the same as offline, just receive messages from ROS
    while (ros::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();
        laser_mapping->Run();
        rate.sleep();
    }

    // Record the end time for the entire process
    auto end_time_total = std::chrono::high_resolution_clock::now();

    // Calculate and print the total processing time
    std::chrono::duration<double> elapsed_time_total = end_time_total - start_time_total;
    double total_time = elapsed_time_total.count(); // in seconds

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    LOG(INFO) << "Total processing time for laser_mapping: " << total_time << " seconds";

    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    faster_lio::Timer::PrintAll(total_time_for_savetrajectory);

    return 0;
}
