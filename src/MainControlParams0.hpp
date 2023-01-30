// Loop Rate //
#define RATE 25.0
// Watch Dog Timer (ms) //
#define WATCHDOG_TIME_LIMIT 700

////////////////
// IO setting //
////////////////
// CAN //
#define CAN0_BAUD_RATE 1000000
#define CAN1_BAUD_RATE 250000
#define CAN2_BAUD_RATE 250000

#define CAN0_ENABLE true
#define CAN1_ENABLE true
#define CAN2_ENABLE true

// Analog //
#define TEENSY_41_ANALOG_PIN_NUM 18

//////////////////
// RMD Settings //
//////////////////
/// Num of RMD ///
#define NUM_OF_RMD 6

/// Direction /// 
#define RMD_DIR_0 1
#define RMD_DIR_1 1
#define RMD_DIR_2 -1
#define RMD_DIR_3 -1
#define RMD_DIR_4 1
#define RMD_DIR_5 -1


////////////////
// RMD Limits //
////////////////
#define RMD_UPPER_0 200
#define RMD_LOWER_0 -250
#define RMD_SPEED_0 900

#define RMD_UPPER_1 240
#define RMD_LOWER_1 -200
#define RMD_SPEED_1 900

#define RMD_UPPER_2 200
#define RMD_LOWER_2 -200
#define RMD_SPEED_2 900

#define RMD_UPPER_3 280
#define RMD_LOWER_3 -210
#define RMD_SPEED_3 900

#define RMD_UPPER_4 150
#define RMD_LOWER_4 -150
#define RMD_SPEED_4 900

#define RMD_UPPER_5 150
#define RMD_LOWER_5 -150
#define RMD_SPEED_5 900

/////////////////////
// GNSS Parameters //
/////////////////////
// Low Pass Filter //
// Cut Off Frequency
#define BODY_GNSS_COF 5
// Gain
#define BODY_GNSS_GAIN 1.0

#define BODY_YAW_GNSS_DIR arav::utils::convert::GnssToPose::DIRECTION::MINUS
#define BODY_YAW_GNSS_AVOID_JUMP true
#define BODY_GNSS_OFFSET 0

////////////////////
// IMU Parameters //
////////////////////
// Low Pass Filter //
// Cut Off Frequency
#define BODY_IMU_COF 15
#define BOOM_IMU_COF 15
#define ARM_IMU_COF 15
#define BUCKET_IMU_COF 15
// Gain
#define BODY_IMU_GAIN 1.0
#define BOOM_IMU_GAIN 1.0
#define ARM_IMU_GAIN 1.0
#define BUCKET_IMU_GAIN 1.0

// Rotate Axis // 
#define BODY_PITCH_IMU_AXIS arav::utils::convert::AccToJoint::ROTATE_AXIS::Y
#define BOOM_IMU_AXIS arav::utils::convert::AccToJoint::ROTATE_AXIS::Z
#define ARM_IMU_AXIS arav::utils::convert::AccToJoint::ROTATE_AXIS::Z
#define BUCKET_IMU_AXIS arav::utils::convert::AccToJoint::ROTATE_AXIS::Z
// Rotate Direction //
#define BODY_PITCH_IMU_DIR arav::utils::convert::AccToJoint::DIRECTION::MINUS
#define BOOM_IMU_DIR arav::utils::convert::AccToJoint::DIRECTION::PLUS
#define ARM_IMU_DIR arav::utils::convert::AccToJoint::DIRECTION::PLUS
#define BUCKET_IMU_DIR arav::utils::convert::AccToJoint::DIRECTION::PLUS
// Avoid atan jump //
#define BODY_PITCH_IMU_AVOID_JUMP true
#define BOOM_IMU_AVOID_JUMP true
#define ARM_IMU_AVOID_JUMP true
#define BUCKET_IMU_AVOID_JUMP true

// Offset (rad)
#define BODY_IMU_OFFSET 0
#define BOOM_IMU_OFFSET -2.6
#define ARM_IMU_OFFSET  -0.0
#define BUCKET_IMU_OFFSET 2.8

#define BOOM_JOINT_OFFSET -1.43+2.93
#define ARM_JOINT_OFFSET 2.65-2.584
#define BUCKET_JOINT_OFFSET -0.601+2.26

/// 
///////////////////////////
// Conversion Parameters //
///////////////////////////
#define BUCKET_FOUR_LINK_A 0.33
#define BUCKET_FOUR_LINK_B 0.505
#define BUCKET_FOUR_LINK_C 0.465
#define BUCKET_FOUR_LINK_D 0.365

////////////////////
// Control Bounds //
////////////////////
#define BODY_BOUND_LOWER -1.0
#define BODY_BOUND_UPPER 1.0
#define BOOM_BOUND_LOWER -1.0
#define BOOM_BOUND_UPPER 0.8
#define ARM_BOUND_LOWER -1.0
#define ARM_BOUND_UPPER 1.0
#define BUCKET_BOUND_LOWER -1.0
#define BUCKET_BOUND_UPPER 1.0


////////////////////
// PID Parameters //
////////////////////
// Rotate Axis //
#define BODY_PID_GAIN_P -100.0
#define BODY_PID_GAIN_I -4.0
#define BODY_PID_GAIN_D -4.0
#define BODY_PID_BOUND_UPPER 70.0
#define BODY_PID_BOUND_LOWER -70.0
#define BOOM_PID_GAIN_P -110.0
#define BOOM_PID_GAIN_I -15.0
#define BOOM_PID_GAIN_D -10.0
#define BOOM_PID_BOUND_UPPER 95.0
#define BOOM_PID_BOUND_LOWER -95.0
#define ARM_PID_GAIN_P -100.
#define ARM_PID_GAIN_I -10.
#define ARM_PID_GAIN_D -6.
#define ARM_PID_BOUND_UPPER 95.0
#define ARM_PID_BOUND_LOWER -95.0
#define BUCKET_PID_GAIN_P 65.
#define BUCKET_PID_GAIN_I 9.0
#define BUCKET_PID_GAIN_D 5.0
#define BUCKET_PID_BOUND_UPPER 90.0
#define BUCKET_PID_BOUND_LOWER -90.0


////////////////
// Automation //
////////////////
#define NUM_OF_AUTOMATION_JOINTS 4
