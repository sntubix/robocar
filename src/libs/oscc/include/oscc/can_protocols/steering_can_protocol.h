/**
 * @file steering_can_protocol.h
 * @brief Steering CAN Protocol.
 *
 */


#ifndef _OSCC_STEERING_CAN_PROTOCOL_H_
#define _OSCC_STEERING_CAN_PROTOCOL_H_


#include <stdint.h>
#include "magic.h"


/*
 * @brief CAN ID representing the range of angle messages.
 *
 */
#define OSCC_STEERING_CAN_ID_INDEX (0x80)


/*
 * @brief Steering enable message (CAN frame) ID.
 *
 */
#define OSCC_STEERING_ENABLE_CAN_ID (0x80)

/*
 * @brief Steering disable message (CAN frame) ID.
 *
 */
#define OSCC_STEERING_DISABLE_CAN_ID (0x81)

/*
 * @brief Steering command message (CAN frame) ID.
 *
 */
#define OSCC_STEERING_TORQUE_COMMAND_CAN_ID (0x82)

/*
 * @brief Steering report message (CAN frame) ID.
 *
 */
#define OSCC_STEERING_REPORT_CAN_ID (0x83)

#define  OSCC_STEERING_ANGLE_COMMAND_CAN_ID (0xB8)
#define OSCC_STEERING_ANGLE_REPORT_CAN_ID (0xB9)

/*
 * @brief Steering report message (CAN frame) length.
 *
 */
#define OSCC_STEERING_REPORT_CAN_DLC (8)

/*
 * @brief Steering report message publishing frequency. [Hz]
 *
 */
#define OSCC_REPORT_STEERING_PUBLISH_FREQ_IN_HZ (50)

/*
 * @brief Enumeration of all possible angle DTCs.
 *
 */
enum
{
    /* DTC bitfield position indicating an invalid sensor value. */
    OSCC_STEERING_DTC_INVALID_SENSOR_VAL = 0,

    /* DTC bitfield position indicating an operator override. */
    OSCC_STEERING_DTC_OPERATOR_OVERRIDE,

    /* Number of possible angle DTCs. */
    OSCC_STEERING_DTC_COUNT
};


#pragma pack(push)
#pragma pack(1)

/**
 * @brief Steering enable message.
 *
 * CAN frame ID: \ref OSCC_STEERING_ENABLE_CAN_ID
 *
 */
typedef struct
{
    uint8_t magic[2]; /*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

     uint8_t reserved[6]; /*!< Reserved. */
} oscc_steering_enable_s;


/**
 * @brief Steering disable message.
 *
 * CAN frame ID: \ref OSCC_STEERING_DISABLE_CAN_ID
 *
 */
typedef struct
{
    uint8_t magic[2]; /*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

    uint8_t reserved[6]; /*!< Reserved. */
} oscc_steering_disable_s;


/**
 * @brief Steering torque command message data.
 *
 * CAN frame ID: \ref OSCC_STEERING_TORQUE_COMMAND_CAN_ID
 *
 */
typedef struct
{
    uint8_t magic[2]; /*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

    float torque_command; /* Steering torque request [-1.0, 1.0] where -1.0 is
                           * max torque in the counterclockwise direction and 1
                           * is max torque in the clockwise direction.
                           * (Note: this is inverse to standard torque direction)
                           */

    uint8_t reserved[2]; /*!< Reserved. */  
} oscc_steering_torque_command_s;


/**
 * @brief Steering report message data.
 *
 * CAN frame ID: \ref OSCC_STEERING_REPORT_CAN_ID
 *
 */
typedef struct
{
    uint8_t magic[2]; /*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

    uint8_t enabled; /*!< Steering controls enabled state.
                      * Zero value means disabled (commands are ignored).
                      * Non-zero value means enabled (no timeouts or overrides have occured). */

    uint8_t operator_override; /*!< Driver override state.
                                * Zero value means there has been no operator override.
                                * Non-zero value means an operator has physically overridden
                                * the system. */

    uint8_t dtcs; /*!< Bitfield of DTCs present in the module. */

    uint8_t reserved[3]; /*!< Reserved. */
} oscc_steering_report_s;


/**
 * @brief Steering angle command message data.
 *
 * CAN frame ID: \ref OSCC_STEERING_ANGLE_COMMAND_CAN_ID
 *
 */
typedef struct
{
//	uint8_t magic[2];
	/*!< Magic number identifying CAN frame as from OSCC.
                       *   Byte 0 should be \ref OSCC_MAGIC_BYTE_0.
                       *   Byte 1 should be \ref OSCC_MAGIC_BYTE_1. */

	uint8_t flag; 	/*!< Boolean flag enabling or disabling the DriveKit PID Steering Angle Control.
                     *   1 "Enable" 0 "Disable" */

	int16_t angle_command; /* Steering angle request [-5500.0, 5500.0] where -5500.0 is
                           * max angle in the counterclockwise direction and 5500.0
                           * is max angle in the clockwise direction. Values are in tenths of degrees.
                           */
	uint16_t max_angular_velocity; /* Steering rate as tenths of degrees/second. 400 is a good value */
} oscc_steering_angle_command_s;


/**
 * @brief Steering angle report message data.
 *
 * CAN frame ID: \ref OSCC_STEERING_ANGLE_REPORT_CAN_ID
 *
 */
typedef struct
{
    uint8_t flag; /*!< steering_angle_report_flags 0 "Enable"; */
    uint8_t error; /*!< steering_angle_report_errors 6 "Command Rejected" 4 "Module Disabled" 2 "Angle Irrational" 1 "Angle Missing" 0 "None"; */
} oscc_steering_angle_report_s;


#pragma pack(pop)


#endif /* _OSCC_STEERING_CAN_PROTOCOL_H_ */
