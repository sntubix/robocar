#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <net/if.h>

#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <unistd.h>
//#include <pthread.h>


#include "oscc.h"
#include "internal/oscc.h"


static int can_socket = -1;
static int sig_handler = 0;


//oscc_result_t oscc_open( unsigned int channel )
//{
//    oscc_result_t result = OSCC_ERROR;
//
//
//    char can_string_buffer[16];
//
//    snprintf( can_string_buffer, 16, "can%u", channel );
//
//    printf( "Opening CAN channel: %s\n", can_string_buffer );
//
//    result = oscc_init_can( can_string_buffer );
//
//
//    return result;
//}

oscc_result_t oscc_open(const char *can_device) {
	oscc_result_t result;

	printf("Opening CAN channel: %s\n", can_device);

	result = oscc_init_can(can_device);


	return result;
}

oscc_result_t oscc_close() {
	oscc_result_t result = OSCC_ERROR;


	if (can_socket != -1) {
		int res = close(can_socket);

		if (res > 0) {
			can_socket = -1;
			while(sig_handler);
			result = OSCC_OK;
		}
	}


	return result;
}

oscc_result_t oscc_poll(int timeout) {
	if (can_socket != -1) {
		struct pollfd pcs;
		pcs.fd = can_socket;
		pcs.events = POLL_IN;
		int ret = poll(&pcs, 1, timeout);

		if (ret == -1) {
			printf("Polling CAN data failed: %s\n", strerror(errno));
			return OSCC_ERROR;
		}

		if ((ret > 0) && (pcs.revents == POLL_IN))
			return OSCC_OK;
	}
	
	return OSCC_ERROR;
}

oscc_result_t oscc_enable(void) {
	oscc_result_t result;


	result = oscc_enable_brakes();

	if (result == OSCC_OK) {
		result = oscc_enable_throttle();

		if (result == OSCC_OK) {
			result = oscc_enable_steering();
		}
	}


	return result;
}

oscc_result_t oscc_disable(void) {
	oscc_result_t result;


	result = oscc_disable_brakes();

	if (result == OSCC_OK) {
		result = oscc_disable_throttle();

		if (result == OSCC_OK) {
			result = oscc_disable_steering();
		}
	}


	return result;
}

oscc_result_t oscc_publish_brake_position(double brake_position) {
	oscc_result_t result;

	oscc_brake_command_s brake_cmd =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	brake_cmd.pedal_command = (float) brake_position;

	result = oscc_can_write(
			OSCC_BRAKE_COMMAND_CAN_ID,
			(void *) &brake_cmd,
			sizeof(brake_cmd));


	return result;
}

oscc_result_t oscc_publish_throttle_position(double throttle_position) {
	oscc_result_t result;

	oscc_throttle_command_s throttle_cmd =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	throttle_cmd.torque_request = (float) throttle_position;

	result = oscc_can_write(
			OSCC_THROTTLE_COMMAND_CAN_ID,
			(void *) &throttle_cmd,
			sizeof(throttle_cmd));


	return result;
}

oscc_result_t oscc_publish_steering_torque(double torque) {
	oscc_result_t result;

	oscc_steering_torque_command_s steering_cmd =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	steering_cmd.torque_command = (float) torque;

	result = oscc_can_write(
			OSCC_STEERING_TORQUE_COMMAND_CAN_ID,
			(void *) &steering_cmd,
			sizeof(steering_cmd));


	return result;
}

oscc_result_t oscc_publish_steering_angle(int angle, int velocity) {
	oscc_result_t result;

	oscc_steering_angle_command_s steering_cmd;
//	=
//			{
//					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
//					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
//			};
	steering_cmd.flag = (uint8_t) 1;
	steering_cmd.angle_command = angle * 10;
	steering_cmd.max_angular_velocity = velocity * 10;

	result = oscc_can_write(
			OSCC_STEERING_ANGLE_COMMAND_CAN_ID,
			(void *) &steering_cmd,
			sizeof(steering_cmd));

	return result;
}

oscc_result_t oscc_subscribe_to_brake_reports(void (*callback)(oscc_brake_report_s *report)) {
	oscc_result_t result = OSCC_ERROR;


	if (callback != NULL) {
		brake_report_callback = callback;
		result = OSCC_OK;
	}


	return result;
}

oscc_result_t oscc_subscribe_to_throttle_reports(void (*callback)(oscc_throttle_report_s *report)) {
	oscc_result_t result = OSCC_ERROR;


	if (callback != NULL) {
		throttle_report_callback = callback;
		result = OSCC_OK;
	}


	return result;
}

oscc_result_t oscc_subscribe_to_steering_reports(void (*callback)(oscc_steering_report_s *report)) {
	oscc_result_t result = OSCC_ERROR;


	if (callback != NULL) {
		steering_report_callback = callback;
		result = OSCC_OK;
	}


	return result;
}

oscc_result_t oscc_subscribe_to_steering_angle_reports(void (*callback)(oscc_steering_angle_report_s *report)) {
	oscc_result_t result = OSCC_ERROR;


	if (callback != NULL) {
		steering_report_angle_callback = callback;
		result = OSCC_OK;
	}


	return result;
}

oscc_result_t oscc_subscribe_to_fault_reports(void (*callback)(oscc_fault_report_s *report)) {
	oscc_result_t result = OSCC_ERROR;


	if (callback != NULL) {
		fault_report_callback = callback;
		result = OSCC_OK;
	}


	return result;
}

oscc_result_t oscc_subscribe_to_obd_messages(void (*callback)(struct can_frame *frame)) {
	oscc_result_t result = OSCC_ERROR;


	if (callback != NULL) {
		obd_frame_callback = callback;
		result = OSCC_OK;
	}


	return result;
}

/* Internal */
oscc_result_t oscc_enable_brakes(void) {
	oscc_result_t result = OSCC_ERROR;

	oscc_brake_enable_s brake_enable =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	result = oscc_can_write(
			OSCC_BRAKE_ENABLE_CAN_ID,
			(void *) &brake_enable,
			sizeof(brake_enable));


	return result;
}

oscc_result_t oscc_enable_throttle(void) {
	oscc_result_t result = OSCC_ERROR;

	oscc_throttle_enable_s throttle_enable =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	result = oscc_can_write(
			OSCC_THROTTLE_ENABLE_CAN_ID,
			(void *) &throttle_enable,
			sizeof(throttle_enable));


	return result;
}

oscc_result_t oscc_enable_steering(void) {
	oscc_result_t result = OSCC_ERROR;

	oscc_steering_enable_s steering_enable =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	result = oscc_can_write(
			OSCC_STEERING_ENABLE_CAN_ID,
			(void *) &steering_enable,
			sizeof(steering_enable));


	return result;
}

oscc_result_t oscc_disable_brakes(void) {
	oscc_result_t result;

	oscc_brake_disable_s brake_disable =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	result = oscc_can_write(
			OSCC_BRAKE_DISABLE_CAN_ID,
			(void *) &brake_disable,
			sizeof(brake_disable));


	return result;
}

oscc_result_t oscc_disable_throttle(void) {
	oscc_result_t result;

	oscc_throttle_disable_s throttle_disable =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	result = oscc_can_write(
			OSCC_THROTTLE_DISABLE_CAN_ID,
			(void *) &throttle_disable,
			sizeof(throttle_disable));


	return result;
}

oscc_result_t oscc_disable_steering(void) {
	oscc_result_t result;

	oscc_steering_disable_s steering_disable =
			{
					.magic[0] = (uint8_t) OSCC_MAGIC_BYTE_0,
					.magic[1] = (uint8_t) OSCC_MAGIC_BYTE_1
			};

	result = oscc_can_write(
			OSCC_STEERING_DISABLE_CAN_ID,
			(void *) &steering_disable,
			sizeof(steering_disable));


	return result;
}

//pthread_mutex_t __oscc_m_ = PTHREAD_MUTEX_INITIALIZER;

void oscc_update_status() {
	sig_handler = 1;
	int saved_errno = errno;

	struct can_frame rx_frame;
	memset(&rx_frame, 0, sizeof(rx_frame));

	if (can_socket != -1) {
		ssize_t ret = read(can_socket, &rx_frame, CAN_MTU);

		while (ret > 0) {
			if ((rx_frame.data[0] == OSCC_MAGIC_BYTE_0)
				&& (rx_frame.data[1] == OSCC_MAGIC_BYTE_1)) {
				if (rx_frame.can_id == OSCC_STEERING_REPORT_CAN_ID) {
					oscc_steering_report_s *steering_report =
							(oscc_steering_report_s *) rx_frame.data;

					if (steering_report_callback != NULL) {
						steering_report_callback(steering_report);
					}
				} else if (rx_frame.can_id == OSCC_THROTTLE_REPORT_CAN_ID) {
					oscc_throttle_report_s *throttle_report =
							(oscc_throttle_report_s *) rx_frame.data;

					if (throttle_report_callback != NULL) {
						throttle_report_callback(throttle_report);
					}
				} else if (rx_frame.can_id == OSCC_BRAKE_REPORT_CAN_ID) {
					oscc_brake_report_s *brake_report =
							(oscc_brake_report_s *) rx_frame.data;

					if (brake_report_callback != NULL) {
						brake_report_callback(brake_report);
					}
				} else if (rx_frame.can_id == OSCC_FAULT_REPORT_CAN_ID) {
					oscc_fault_report_s *fault_report =
							(oscc_fault_report_s *) rx_frame.data;

					if (fault_report_callback != NULL) {
						fault_report_callback(fault_report);
					}
				}
			} else if (rx_frame.can_id == OSCC_STEERING_ANGLE_REPORT_CAN_ID) {
				oscc_steering_angle_report_s *steering_report =
						(oscc_steering_angle_report_s *) rx_frame.data;

				if (steering_report_angle_callback != NULL) {
					steering_report_angle_callback(steering_report);
				}
			} else {
				if (obd_frame_callback != NULL) {
					obd_frame_callback(&rx_frame);
				}
			}

			ret = read(can_socket, &rx_frame, CAN_MTU);
		}
	}
	//pthread_mutex_unlock(&__oscc_m_);

	errno = saved_errno;
	sig_handler = 0;
}

oscc_result_t oscc_can_write(long id, void *msg, unsigned int dlc) {
//	printf("CANwrite: %ld, %d\n", id, dlc);
	oscc_result_t result = OSCC_ERROR;


	if (can_socket != -1) {
		struct can_frame tx_frame;

		memset(&tx_frame, 0, sizeof(tx_frame));
		tx_frame.can_id = id;
		tx_frame.can_dlc = dlc;
		memcpy(tx_frame.data, msg, dlc);
		ssize_t ret = write(can_socket, &tx_frame, sizeof(tx_frame));

		if (ret > 0) {
			result = OSCC_OK;
		} else {
			printf("Could not write to socket: %s\n", strerror(errno));
		}
	}


	return result;
}

oscc_result_t oscc_async_enable(int socket) {
	oscc_result_t result = OSCC_ERROR;

	int ret = fcntl(socket, F_SETOWN, getpid());

	if (ret < 0) {
		printf("Setting owner process of socket failed: %s\n", strerror(errno));
	} else {
		result = OSCC_OK;
	}


	if (result == OSCC_OK) {
		ret = fcntl(socket, F_SETFL, FASYNC | O_NONBLOCK);

		if (ret < 0) {
			printf("Setting nonblocking asynchronous socket I/O failed: %s\n", strerror(errno));

			result = OSCC_ERROR;
		}
	}


	return result;
}

oscc_result_t oscc_init_can(const char *can_channel) {
	int result = OSCC_ERROR;
	int ret;

//    set_thread_SIG_handler(oscc_update_status);
//    install_SIG_handlers(SIGIO);

	struct sigaction sig;

	memset(&sig, 0, sizeof(sig));
	sigemptyset(&sig.sa_mask);
	sig.sa_flags = SA_RESTART;
	sig.sa_handler = oscc_update_status;
	sigaction(SIGIO, &sig, NULL);

	int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (sock < 0) {
		printf("Opening CAN socket failed: %s\n", strerror(errno));
	} else {
		result = OSCC_OK;
	}


	struct ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));

	if (result == OSCC_OK) {
		strncpy(ifr.ifr_name, can_channel, IFNAMSIZ);

		ret = ioctl(sock, SIOCGIFINDEX, &ifr);

		if (ret < 0) {
			printf("Finding CAN index failed: %s\n", strerror(errno));

			result = OSCC_ERROR;
		}
	}


	if (result == OSCC_OK) {
		struct sockaddr_can can_address;

		memset(&can_address, 0, sizeof(can_address));
		can_address.can_family = AF_CAN;
		can_address.can_ifindex = ifr.ifr_ifindex;

		ret = bind(
				sock,
				(struct sockaddr *) &can_address,
				sizeof(can_address));

		if (ret < 0) {
			printf("Socket binding failed: %s\n", strerror(errno));

			result = OSCC_ERROR;
		}
	}


	if (result == OSCC_OK) {
		ret = oscc_async_enable(sock);

		if (ret != OSCC_OK) {
			printf("Enabling asynchronous socket I/O failed\n");

			result = OSCC_ERROR;
		}
	}


	if (result == OSCC_OK) {
		/* all prior checks will pass even if a valid interface has not been
		   set up - attempt to write an empty CAN frame to the interface to see
		   if it is valid */
		struct can_frame tx_frame;

		memset(&tx_frame, 0, sizeof(tx_frame));
		tx_frame.can_id = 0;
		tx_frame.can_dlc = 8;

		ssize_t bytes_written = write(sock, &tx_frame, sizeof(tx_frame));

		if (bytes_written < 0) {
			printf("Failed to write test frame to %s: %s\n", can_channel, strerror(errno));

			result = OSCC_ERROR;
		}
	}

	if (result == OSCC_OK) {
		// check that we can receive data from the CAN
		struct pollfd pcs;
		pcs.fd = sock;
		pcs.events = POLL_IN;
		int ret = poll(&pcs, 1, 5000);

		if (ret == -1) {
			printf("Waiting for CAN data failed: %s\n", strerror(errno));
			result = OSCC_ERROR;
		}

		if (ret == 0) {
			printf("Waiting for CAN data timeout, assuming CAN unavailable\n");
			result = OSCC_ERROR;
		}
		
		if(pcs.revents == POLL_IN)
			can_socket = sock;
	}

	return result;
}
