#include "optitrack_listener_ros2/optitrack_listener.hpp"

OptitrackListener::OptitrackListener(): Node("optitrack_listener"){

	timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out",10,
		[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
			timestamp_.store(msg->timestamp);
		}
	);

	declare_parameter("port", 9030);
    declare_parameter("blocking", false);
	declare_parameter("debug", false);

	param_port_ = get_parameter("port").as_int();
    param_blocking_ = get_parameter("blocking").as_bool();
    param_debug_ = get_parameter("debug").as_bool();

	// Publishers
	odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in", 1);
  	
  	timestamp_sample = 0;

	Publisher();
}

/* open socket */
bool OptitrackListener::Open(int port_number) {

	if ( (opt_socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		std::cout << "Listener::Open: error during socket creation!" << std::endl;
		return false;
	}
	else
		std::cout << "Listener::Open completed!" << std::endl;

	/* blocking or no-blocking */
	if (param_blocking_)
		fcntl(opt_socket,F_SETFL,O_NONBLOCK);

	memset((char *) &si_me, 0, sizeof(si_me));

	/* allow connections to any address port */
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port_number);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(opt_socket, (struct sockaddr*)&si_me, sizeof(si_me))==-1) {
		std::cout << "Listener::Open: error during bind!" << std::endl;
		return false;
	}
	return true;
}

void OptitrackListener::Publisher(){

	if ( !Open(param_port_) ) {
		cout << "Socket: opening failed!" << endl;
	}
	else{

		auto timer_callback = [this]() -> void {

			if(rclcpp::ok()){
					
				slen=sizeof(si_other);
				rlen = recvfrom(opt_socket, buffer, 1024, 0,(struct sockaddr*)&si_other, (socklen_t*)&slen);
				char pck_type = (char)buffer[0];

				if (pck_type == 'O') { //Other markers
					oth_markers = *((struct optitrack_other_markers_udp_packet *)(buffer+1));
					if( param_debug_ )
						printf("Other marker (iFrame %i, Latency %f ms, num_markers %i)\n", oth_markers.iFrame, oth_markers.Latency, oth_markers.nMarkers);

					int index = sizeof(struct optitrack_other_markers_udp_packet)+1;
					for (int j = 0; j<oth_markers.nMarkers; j++) {
						struct optitrack_marker_udp_packet marker;
						marker = *((struct optitrack_marker_udp_packet *)(buffer+index));
						index += sizeof(optitrack_marker_udp_packet);
						if (param_debug_)
							printf(" - marker #%i: %f, %f, %f m\n", marker.ID, marker.x, marker.y, marker.z);
					}
				}
				else if (pck_type == 'T') { //Trackable

					rb = *((struct optitrack_rigid_body_udp_packet *)(buffer+1));

					Eigen::Matrix3d R_o;
					R_o << 0, 0, 1, 
						1, 0, 0,
						0, 1, 0;

					//---Raw input
					Eigen::Vector3d p_opt; // = Zeros;
					Eigen::Vector4d q_opt; // = Zeros;
					p_opt << rb.x, rb.y, rb.z;
					q_opt << rb.qw, rb.qx, rb.qy, rb.qz;
					
					Eigen::Matrix3d R_opt = utilities::QuatToMat( q_opt );

					// Output frame NED

					Eigen::Matrix3d R_o_ned;
					R_o_ned <<  0,  0, 1,
							-1,  0, 0,
								0, -1, 0;

					//Rotate optitrack data
					Eigen::Vector3d p_opt2_ned = R_o_ned*p_opt;
					Eigen::Matrix3d R_opt2_ned = R_o_ned*R_opt*R_o.transpose();
					Eigen::Vector4d q_opt2_ned = utilities::MatToQuat(R_opt2_ned);

					odometry_.local_frame = 0; //NED
					odometry_.x = p_opt2_ned[0];
					odometry_.y = p_opt2_ned[1];
					odometry_.z = p_opt2_ned[2];

					odometry_.q = {float(q_opt2_ned[0]), float(q_opt2_ned[1]), float(q_opt2_ned[2]), float(q_opt2_ned[3])};
				
					timestamp_sample++;
					
					odometry_.timestamp = timestamp_.load();
					odometry_.timestamp_sample = timestamp_.load();
								
					//std::cout << "P: " << position_.t() << std::endl;
				
					odom_publisher_->publish(odometry_);

					int index = sizeof(struct optitrack_rigid_body_udp_packet)+1;
					for (int j = 0; j<rb.nMarkers; j++) {
						struct optitrack_marker_udp_packet marker;
						marker = *((struct optitrack_marker_udp_packet *)(buffer+index));
						index += sizeof(optitrack_marker_udp_packet);
						printf(" - marker #%i: %f, %f, %f m\n", marker.ID, marker.x, marker.y, marker.z);
					}
				}
			}
	
		};
		timer_ = this->create_wall_timer(10ms, timer_callback); // 100 Hz
	}
}


int main(int argc, char* argv[]) {
	std::cout << "Starting optitrack_listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<OptitrackListener>());

	rclcpp::shutdown();
	return 0;
}


