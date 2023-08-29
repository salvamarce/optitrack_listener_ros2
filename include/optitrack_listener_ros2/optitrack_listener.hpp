#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <rclcpp/rclcpp.hpp>
#include <optitrack_listener_ros2/utils.h>

#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <Eigen/Eigen>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#ifndef O_LISTENER_H_
#define O_LISTENER_H_

#define OPTITRACK_BUF_SIZE 1000
#define OPTITRACK_PORT_DEF 9030


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;

struct optitrack_rigid_body_udp_packet {
	int ID;
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
	int iFrame;
	float Latency;
	int nMarkers;
};
struct optitrack_other_markers_udp_packet {
	int iFrame;
	float Latency;
	int nMarkers;
};
struct optitrack_marker_udp_packet {
	int ID;
	float x;
	float y;
	float z;
};

class OptitrackListener : public rclcpp::Node {
public:
	
    OptitrackListener();
    bool Open(int port_number);
	
private:

	void Publisher();

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr odom_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	std::atomic<uint64_t> timestamp_sample;
	
	VehicleVisualOdometry odometry_{};

    int opt_socket;
    std::stringstream intToString;
	int rlen, slen;
	unsigned char buffer[1024];
    sockaddr_in si_me, si_other;
	optitrack_rigid_body_udp_packet optitrack_data;
    int param_port_;
	bool param_blocking_;
	bool param_debug_;
	struct optitrack_rigid_body_udp_packet rb;
	struct optitrack_other_markers_udp_packet oth_markers;

};

#endif /* O_LISTENER_H_ */

