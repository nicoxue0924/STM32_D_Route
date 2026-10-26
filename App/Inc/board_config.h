#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Hardware: STM32F103ZET6, YB-DSF01 V1.0/V1.1, AT8236 x4.
 * Index 0..3 means board sockets M1..M4. Factory convention: M1/M2 left,
 * M3/M4 right. The human has chosen this provisional wiring convention.
 * Wheel diameter is human-measured; output CPR and yaw/slip still need calibration. */
#define CONTROL_PERIOD_MS          10U
#define CONTROL_MAX_INTERVAL_MS    50U
#define DEBUG_PERIOD_MS            100U
#define KEY_DEBOUNCE_MS            25U

#define MOTOR_COUNT                4U
#define MOTOR_PWM_PERIOD_COUNTS    3600U
#define MOTOR_PWM_LIMIT_COUNTS     2800U
/* Factory motor example uses 2000/3600 as its starting dead zone. */
#define MOTOR_DEADZONE_COUNTS      2000U
#define MOTOR_DIRECTION_SIGNS      {-1, -1, 1, 1}
#define ENCODER_DIRECTION_SIGNS    { 1,  1,-1,-1}
#define LEFT_FRONT_MOTOR           0U
#define LEFT_REAR_MOTOR            1U
#define RIGHT_FRONT_MOTOR          2U
#define RIGHT_REAR_MOTOR           3U

/* Human measured wheel diameter: 41.5 mm (2026-10-23).
 * PROVISIONAL: 1040 counts/rev is from the vendor's supplied 450-RPM example,
 * NOT confirmed specifications of the user's 310 motors.
 * COUNTS_PER_REV is the gearbox OUTPUT wheel revolution, with x4 decoding.
 * Confirm output CPR / loaded rolling distance during subsequent calibration. */
#define WHEEL_DIAMETER_MM          41.5f
#define WHEEL_COUNTS_PER_REV       1040.0f
#define SPEED_KP                   0.8f
#define SPEED_KI                   6.0f
#define SPEED_KD                   0.0f
#define SPEED_ACCEL_MM_S2          800.0f
#define SPEED_DECEL_MM_S2          1600.0f
/* V8 doubled target rates: arc outer speed is about 491 mm/s.
 * 600 leaves room for the existing synchronization correction.
 * PWM limits and acceleration/deceleration settings are unchanged. */
#define SPEED_MAX_MM_S             600.0f
/* V6 removes encoder stall/reverse automatic fault stops.
 * Encoder feedback still controls wheel speed and route distance. */

/* Gray module: J15 pin2 PF13->AD2; pin3 PF14->AD1; pin4 PF15->AD0;
 * pin5 PG0<-OUT. Address 000..111 selects X1..X8 respectively.
 * ACTIVE_LEVEL: raw OUT level when seeing the chosen track line.
 * X1_IS_LEFT: channel X1 is physically on the left viewed along travel.
 * Both switches must match the actual surface and mounting orientation. */
/* V10 MPU6050 on U7 PB10/PB11; flat mounting, Z arrow upward.
 * Right turn must read positive yaw; flip SIGN only if a hand-rotation check
 * proves otherwise. Not a compatible driver for UART AHRS modules. */
#define IMU_YAW_SIGN               -1.0f
#define IMU_CALIBRATION_SAMPLES     300U
#define IMU_CALIBRATION_VARIANCE_DPS2 0.25f
#define IMU_MAX_SAMPLE_AGE_MS       30U
#define HEADING_STRAIGHT_KP         8.0f
#define HEADING_STRAIGHT_KD         1.0f
#define HEADING_STRAIGHT_MAX_MM_S   80.0f
#define HEADING_BOUNDARY_MAX_DEG    6.0f
#define HEADING_BOUNDARY_SLEW_DPS   60.0f
#define HEADING_TURN_ANGLE_KP       4.0f
#define HEADING_TURN_RATE_KP        1.0f
#define HEADING_TURN_REVERSE_MAX_DPS 30.0f
#define HEADING_TURN_TOLERANCE_DEG  2.0f
#define HEADING_TURN_EXIT_RATE_DPS  10.0f
#define GRAY8_ACTIVE_LEVEL         1U
#define GRAY8_X1_IS_LEFT           0U
#define GRAY8_SETTLE_US            100U
#define GRAY8_MARKER_MIN_ACTIVE    5U

/* Problem G-1, clockwise Start-A-B-C-D-Start, lane-center initial geometry.
 * Outer field 1800 mm, outer radius 250 mm, road width 250 mm.
 * Start line x=1300: line-to-arc-entry is 1300-250=1050 mm.
 * Human aligns REAR with start line. Estimate center 212/2=106 mm ahead:
 * first wheel travel = 1050-106=944 mm. Measure offset if center differs.
 * Side tangents 1800-2*250=1300 mm; final straight 250 mm.
 * Center radius 250-250/2=125 mm. All lengths are in millimeters. */
#define ROUTE_FIRST_MM             944.0f
#define ROUTE_SIDE_MM              1300.0f
#define ROUTE_TOP_MM               1300.0f
#define ROUTE_FINISH_NOMINAL_MM     250.0f
#define ROUTE_ARC_RADIUS_MM         125.0f
/* Human overall dimensions: length 212 mm, outer width 159 mm.
 * Outer width is NOT wheel-center track or effective skid-steer track.
 * Keep the model track as a reference; its yaw correction is calibrated below. */
#define ROUTE_EFFECTIVE_TRACK_MM    160.0f
#define ROUTE_ARC_DISTANCE_SCALE    1.0f
/* V7: human reports the V6 run turned about 96 deg.
 * Initial correction: 1.68637124*(90/96)=1.58097304.
 * Straight lengths and nominal center arc travel stay unchanged.
 * Combined with a new arc handoff; physical yaw still needs confirmation. */
#define ROUTE_TURN_DIFFERENTIAL_GAIN 1.58097304f
#define ROUTE_STRAIGHT_SPEED_MM_S   400.0f
/* V8 initial trim for drift already present on the first straight.
 * Positive = slightly more left travel / less right travel, steering right.
 * Straight feed-forward only; mean encoder travel sets segment completion. */
#define ROUTE_STRAIGHT_TRIM         0.005f
/* V9 lane-boundary avoidance on straight/finish segments only.
 * position < 0: left boundary -> left faster/right slower -> steer right.
 * -1 avoids a boundary; +1 would follow a line (not the current lane mode).
 * Limit each side correction to 80 mm/s; keep both wheels moving forward. */
#define ROUTE_LINE_DIRECTION        -1.0f
#define ROUTE_LINE_KP               28.0f
#define ROUTE_LINE_MAX_MM_S         80.0f
/* V8 doubles V7 nominal speeds; distance and turn gain remain unchanged. */
#define ROUTE_ARC_CENTER_SPEED_MM_S 244.0f
#define ROUTE_FINISH_SPEED_MM_S     240.0f
#define ROUTE_END_SPEED_MM_S        200.0f
#define ROUTE_ARC_END_CENTER_MM_S   122.0f
#define ROUTE_SLOW_ZONE_MM          60.0f
#define ROUTE_GOAL_TOLERANCE_MM     2.0f
#define ROUTE_SYNC_KP               1.2f
#define ROUTE_SYNC_MAX_MM_S         40.0f
/* Final-stage gating rejects earlier bars. Arm immediately at its entry so
 * a forward sensor crossing before 150 mm cannot be missed. */
#define ROUTE_FINISH_ARM_MM         0.0f
#define ROUTE_FINISH_MAX_MM         (ROUTE_FINISH_NOMINAL_MM+200.0f)
#define ROUTE_SEGMENT_MARGIN_MS     8000U
#define ROUTE_MAX_RUN_MS            120000U
#define CROSSLINE_CLEAR_MS          100U
#define CROSSLINE_CONFIRM_MS        30U
#define STOP_SETTLE_SPEED_MM_S     15.0f
#define STOP_SETTLE_MS             150U
#define STOP_MAX_BRAKING_MS        1600U
#define STOP_BRAKE_HOLD_MS         400U

#endif
