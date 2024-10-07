#include <arpa/inet.h>
#include <libkern/OSByteOrder.h>
#include <machine/endian.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "redis/keys/chai_haptic_devices_driver.h"
#include "timer/LoopTimer.h"

using namespace std;
using namespace Eigen;
using namespace Sai2Common::ChaiHapticDriverKeys;

namespace {

// signal handling
bool runloop = true;
void sighandler(int) { runloop = false; }

bool gripper_was_pressed = false;

// loop timer parameters
const double haptic_control_freq = 1000.0;
const double UDP_send_freq = 40.0;
const int number_of_control_loops_to_average =
	haptic_control_freq / UDP_send_freq;
int averaging_counter = 1;

// haptic device parameters
const Vector3d device_home_position = Vector3d(0, 0, 0);
const double max_distance_translation = 0.015;
const double max_angle_rotation = 60.0 / 180.0 * M_PI;

const double admittance_p_gain_pos = 350.0;
const double admittance_d_gain_pos = 15.0;
const double admittance_p_gain_ori = 0.3;
const double admittance_d_gain_ori = 0.1;

// network related parameters
const int port = 8080;
const string ip_address = "127.0.0.1";

void encodeDouble(double value, unsigned char* buffer) {
	// Copy double to buffer
	std::memcpy(buffer, &value, sizeof(value));

	// Check system endianness
	// if (__BYTE_ORDER == __LITTLE_ENDIAN) {
	if (OSHostByteOrder() == OSLittleEndian) {
		// If little-endian, reverse the byte order
		for (int i = 0; i < 4; ++i) {
			std::swap(buffer[i], buffer[7 - i]);
		}
	}
}

}  // namespace

int main() {
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// redis client for haptic communication with driver
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// UDP socket setup
	unsigned char buffer[24];
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		std::cerr << "UDP socket creation failed" << std::endl;
		return -1;
	}
	// Define the server address
	struct sockaddr_in server_addr;
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);
	server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

	// create haptic controller
	Sai2Primitives::HapticDeviceController::DeviceLimits device_limits(
		redis_client.getEigen(createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_DAMPING_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_FORCE_KEY_SUFFIX, 0)));
	Affine3d device_home_pose = Affine3d(Translation3d(device_home_position));
	auto haptic_controller =
		make_shared<Sai2Primitives::HapticDeviceController>(
			device_limits, Affine3d::Identity(), device_home_pose);
	haptic_controller->setHapticControlType(
		Sai2Primitives::HapticControlType::HOMING);
	haptic_controller->disableOrientationTeleop();
	redis_client.setBool(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0),
						 true);

	Sai2Primitives::HapticControllerInput haptic_input;
	Sai2Primitives::HapticControllerOtuput haptic_output;

	Matrix3d device_reference_rotation = Matrix3d::Identity();

	// values to send
	double x_command = 0.0;
	double y_command = 0.0;
	double angle_command = 0.0;

	// setup redis communication
	redis_client.addToSendGroup(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
								haptic_output.device_command_force);
	redis_client.addToSendGroup(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
								haptic_output.device_command_moment);

	redis_client.addToReceiveGroup(createRedisKey(POSITION_KEY_SUFFIX, 0),
								   haptic_input.device_position);
	redis_client.addToReceiveGroup(createRedisKey(ROTATION_KEY_SUFFIX, 0),
								   haptic_input.device_orientation);
	redis_client.addToReceiveGroup(
		createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_linear_velocity);
	redis_client.addToReceiveGroup(
		createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_angular_velocity);

	bool gripper_pressed = false;
	redis_client.addToReceiveGroup(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
								   gripper_pressed);

	// create a timer
	Sai2Common::LoopTimer controlTimer(1000.0, 1e6);

	while (runloop) {
		// wait for next scheduled loop
		controlTimer.waitForNextLoop();

		// read haptic device state from redis
		redis_client.receiveAllFromGroup();
		haptic_output = haptic_controller->computeHapticControl(haptic_input);
		redis_client.sendAllFromGroup();

		// switch to admittance control after homing
		if (haptic_controller->getHapticControlType() ==
				Sai2Primitives::HapticControlType::HOMING &&
			haptic_controller->getHomed()) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::FORCE_MOTION);
			haptic_controller->setDeviceControlGains(
				admittance_p_gain_pos, admittance_d_gain_pos,
				admittance_p_gain_ori, admittance_d_gain_ori);
			haptic_controller->enablePlaneGuidance(Vector3d::Zero(),
												   Vector3d::UnitZ());
			// haptic_controller->disableHapticWorkspaceVirtualLimits();
		}

		// compute Toro commands
		double x_command_scaled_this_loop = (device_home_pose.translation()(0) -
											 haptic_input.device_position(0)) /
											max_distance_translation;
		double y_command_scaled_this_loop = (device_home_pose.translation()(1) -
											 haptic_input.device_position(1)) /
											max_distance_translation;
		if (fabs(x_command_scaled_this_loop) > 1.0) {
			x_command_scaled_this_loop =
				x_command_scaled_this_loop / fabs(x_command_scaled_this_loop);
		}
		if (fabs(y_command_scaled_this_loop) > 1.0) {
			y_command_scaled_this_loop =
				y_command_scaled_this_loop / fabs(y_command_scaled_this_loop);
		}

		// use angle only is gripper is pressed
		double angle_command_scaled_this_loop = 0.0;
		if (gripper_pressed) {
			if (!gripper_was_pressed) {
				device_reference_rotation = haptic_input.device_orientation;
				gripper_was_pressed = true;
			}

			AngleAxisd orientation_error(device_reference_rotation.transpose() *
										 haptic_input.device_orientation);

			double angle_error =
				orientation_error.angle() * orientation_error.axis()(2);

			angle_command_scaled_this_loop = angle_error / max_angle_rotation;
			if (fabs(angle_command_scaled_this_loop) > 1.0) {
				angle_command_scaled_this_loop =
					angle_command_scaled_this_loop /
					fabs(angle_command_scaled_this_loop);
			}
		} else {
			gripper_was_pressed = false;
		}

		// average commands at 40 Hz
		x_command +=
			(x_command_scaled_this_loop - x_command) / averaging_counter;
		y_command +=
			(y_command_scaled_this_loop - y_command) / averaging_counter;
		angle_command += (angle_command_scaled_this_loop - angle_command) /
						 averaging_counter;

		averaging_counter++;

		if (averaging_counter > number_of_control_loops_to_average) {
			averaging_counter = 1;

			// cout << "x_command: " << x_command << endl;
			// cout << "y_command: " << y_command << endl;
			// cout << "angle_command: " << angle_command << endl;
			// cout << endl;

			// send commands to UDP
			encodeDouble(x_command, buffer);
			encodeDouble(y_command, buffer + 8);
			encodeDouble(angle_command, buffer + 16);

			ssize_t sent_bytes =
				sendto(sockfd, buffer, sizeof(buffer), 0,
					   (struct sockaddr*)&server_addr, sizeof(server_addr));

			if (sent_bytes != sizeof(buffer)) {
				std::cerr << "Failed to send all bytes" << std::endl;
			}
		}
	}

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();

	close(sockfd);

	return 0;
}
