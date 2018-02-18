/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import frclib.FrcPixyCam;

public class RobotInfo
{
    //
    // Compiler switches
    //

    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54*12.0;
    public static final double FIELD_WIDTH                      = 27*12.0;

    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 38.0;
    public static final double ROBOT_WIDTH                      = 35.0;
    public static final double ROBOT_HEIGHT                     = 24.0;

    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK              = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK             = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;
    
    //
    // Teleop constants
    //
    public static final double TELEOP_WINCH_POWER               = 0.6; // TODO: This needs to be calibrated
    public static final double TELEOP_GRABBER_POWER             = 0.6; // TODO: This needs to be calibrated
    public static final boolean TELEOP_ENABLE_DRIVE_INVERSION   = false;
    
    //
    // Analog Input ports.
    //
    public static final int AIN_FRONT_SONAR_SENSOR              = 0;
    public static final int AIN_LEFT_SONAR_SENSOR               = 1;
    public static final int AIN_RIGHT_SONAR_SENSOR              = 2;
    public static final int AIN_PRESSURE_SENSOR                 = 3;
    //
    // Digital Input ports.
    //
    public static final int DIN_FRONT_SONAR_PING                = 0;
    public static final int DIN_LEFT_SONAR_PING                 = 1;
    public static final int DIN_RIGHT_SONAR_PING                = 2;
    public static final int DIN_LEFT_PROXIMITY_SENSOR           = 3;
    public static final int DIN_RIGHT_PROXIMITY_SENSOR          = 4;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTWHEEL                = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL               = 4;    // 40A: Yellow
    public static final int CANID_LEFTREARWHEEL                 = 5;    // 40A: Green
    public static final int CANID_RIGHTREARWHEEL                = 6;    // 40A: Blue
    public static final int CANID_WINCH_MASTER                  = 7;    // 40A: Purple
    public static final int CANID_WINCH_SLAVE                   = 8;    // 40A: Gray
    public static final int CANID_ELEVATOR                      = 9;    // 30A: White
    public static final int CANID_LEFT_PICKUP                   = 10;   // 30A: Orange
    public static final int CANID_RIGHT_PICKUP                  = 12;   // 30A: Green

    public static final int CANID_PDP                           = 16;
    public static final int CANID_PCM1                          = 17;
    public static final int CANID_PCM2                          = 18;

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER               = 0;    // 20A: White

    //
    // Solenoid channels.
    //
    public static final int SOL_CUBEPICKUP_ARM_RETRACT          = 0;
    public static final int SOL_CUBEPICKUP_ARM_EXTEND           = 1;
    public static final int SOL_CUBEPICKUP_CLAW_EXTEND          = 2;
    public static final int SOL_CUBEPICKUP_CLAW_RETRACT         = 3;
    public static final int SOL_LEFT_FLIPPER_RETRACT            = 4;
    public static final int SOL_LEFT_FLIPPER_EXTEND             = 5;
    public static final int SOL_RIGHT_FLIPPER_RETRACT           = 6;
    public static final int SOL_RIGHT_FLIPPER_EXTEND            = 7;

//    public static final int SOL_TARGET_FOUND_LED                = 6;    // White LED
//    public static final int SOL_TARGET_ALIGNED_LED              = 7;    // Blue LED

    //
    // Miscellaneous sensors and devices.
    //
    public static final int USBCAM_WIDTH                        = 320;
    public static final int USBCAM_HEIGHT                       = 240;
    public static final int USBCAM_FRAME_RATE                   = 15;
    public static final int USBCAM_BRIGHTNESS                   = 20;

    //
    // Vision subsystem.
    //
    public static final int PIXYCAM_WIDTH                       = 320;
    public static final int PIXYCAM_HEIGHT                      = 200;
    public static final int PIXY_POWER_CUBE_SIGNATURE           = 1;
    public static final int PIXY_BRIGHTNESS                     = 80;
    public static final double PIXY_CAM_OFFSET                  = 8.0;
    public static final PixyVision.Orientation PIXY_ORIENTATION = PixyVision.Orientation.NORMAL_LANDSCAPE;
    public static final int PIXYCAM_I2C_ADDRESS                 = FrcPixyCam.DEF_I2C_ADDRESS;

    //
    // DriveBase subsystem.
    //
    
    public static final double GYRO_ASSIST_KP                   = 0.3;
    public static final double GYRO_ASSIST_SCALE                = 0.5;

    public static final double DRIVE_STALL_TIMEOUT              = 0.5;
    public static final double DRIVE_SLOW_XSCALE                = 3.0;
    public static final double DRIVE_SLOW_YSCALE                = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE             = 3.0;

    // 2017-03-21: 0.0152347136491642, 0.15, 0.0, 0.0
    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.0152347136491642;
    public static final double ENCODER_X_KP                     = 0.15;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.0;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 1.0;

    // 2017-04-06: 0.01778656, 0.04, 0.0, 0.004
  //OLD ENCODER Y INCHES PER COUNT: 0.02155312564705882352941176470588;//0.01778656;
    public static final double ENCODER_Y_INCHES_PER_COUNT       = 0.016789667896679;
    public static final double ENCODER_Y_KP                     = 0.04;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.0077;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 1.0;

    // 2017-04-05: 0.03, 0.0, 0.003
    public static final double GYRO_TURN_KP                     = 0.03;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.003;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;
    public static final double GYRO_TURN_SMALL_THRESHOLD        = 10.0;
    public static final double GYRO_TURN_SMALL_KP               = 0.06;
    public static final double GYRO_TURN_SMALL_KI               = 0.0;
    public static final double GYRO_TURN_SMALL_KD               = 0.0018;

    // 2017-03-12: 0.01, 0.0, 0.0
    // 2017-03-14: 0.0165, 0.0, 0.002
    public static final double SONAR_MILLIVOLTS_PER_INCH        = 0.0098;
    public static final double SONAR_KP                         = 0.0165;
    public static final double SONAR_KI                         = 0.0;
    public static final double SONAR_KD                         = 0.002;
    public static final double SONAR_KF                         = 0.0;
    public static final double SONAR_TOLERANCE                  = 1.0;

    // 2017-03-12: 0.025, 0.0, 0.0001
    // 2017-03-14: 0.025, 0.0, 0.00015
    public static final double VISION_TURN_KP                   = 0.025;
    public static final double VISION_TURN_KI                   = 0.0;
    public static final double VISION_TURN_KD                   = 0.00015;
    public static final double VISION_TURN_KF                   = 0.0;
    public static final double VISION_TURN_TOLERANCE            = 1.0;

    //
    // Winch subsystem.
    //
    public static final double WINCH_MOTOR_POWER_SCALE          = 1.0;      //Disable slow down, used to be 0.6
//    public static final double WINCH_POSITION_SCALE             = 0.0026767420949418;
//    public static final double WINCH_MOTOR_CURRENT_THRESHOLD    = 20.0;
//    public static final double WINCH_HEIGHT_THRESHOLD           = 28.0;
//    public static final double WINCH_SPIKE_TIMEOUT              = 0.5;
//    public static final double WINCH_TILT_THRESHOLD             = 35.0;
    //
    // CubePickup subsystem.
    //
    public static final double GRABBER_CURRENT_THRESHOLD        = 40.0;     // Need to be tuned.
    public static final double GRABBER_CURRENT_OFFSET           = 10.0;

    //
    // Elevator subsystem.
    //
    public static final double ELEVATOR_INCHES_PER_COUNT        = 0.00577778;   // 39 inches in 6750 ticks
    public static final double ELEVATOR_KP                      = 0.3;                // this too
    public static final double ELEVATOR_KI                      = 0.0;                // hopefully not this
    public static final double ELEVATOR_KD                      = 0.0;                // this too
    public static final double ELEVATOR_TOLERANCE               = 0.5;                // this too
    public static final double ELEVATOR_MIN_HEIGHT              = 8.0;
    public static final double ELEVATOR_MAX_HEIGHT              = 85.0;             //need calibration
    public static final double ELEVATOR_MID_HEIGHT              = 4.0;
    public static final double ELEVATOR_CAL_POWER               = 0.3;                // this too
    public static final double ELEVATOR_FLOOR_PICKUP_HEIGHT     = 0.0;                // Lowest point on elevator
    // this used to be 0.04735821
    public static final double ELEVATOR_GRAVITY_COMPENSATION    = 0.08;               // Tuned during testing
    public static final double ELEVATOR_POSITION_OFFSET         = 8.0;
    
    //
    // AutoAssist subsystem.
    //
    public static final double PORTAL_ALIGN_STRAFE_DIST         = 36.0; // 3 feet
    public static final double AUTO_PICKUP_MOVE_POWER           = 0.6; // 60% power
    public static final double FIND_CUBE_ANGLE_TOLERANCE        = 15.0; // 15 degrees
    public static final double FIND_CUBE_STRAFE_POWER           = 0.6; // 60% power

    //
    // CmdPowerUpAuto variables.
    //
    public static final double AUTO_DISTANCE_TO_SWITCH = 158.0;
    public static final double FINAL_FRONT_SCALE_APPROACH_DISTANCE = 64.0;
    public static final double FINAL_SIDE_SCALE_APPROACH_DISTANCE = 24.0;
    public static final double RIGHT_SWITCH_LOCATION = 102.0;
    public static final double LEFT_SWITCH_LOCATION = -102.0;
    public static final double SCALE_TARGET_HEIGHT = 60.0;
    public static final double ADVANCE_TO_SECOND_CUBE_DISTANCE = 72.0;
    public static final double STRAFE_TO_SECOND_CUBE_DISTANCE = 31.0;
    public static final double SECOND_CUBE_BACKUP_DISTANCE = 30.0;
    public static final double SCALE_FRONT_POSITION = 75.0;
    public static final double SCALE_SIDE_POSITION = 120.0;
    public static final double FIRST_ELEVATOR_HEIGHT = 30.0;
    public static final double ADVANCE_AROUND_SCALE_DISTANCE = 94.0;
    
    //
    // FrcAuto constants.
    //
    public static final double FWD_DISTANCE_1 = 4.0;
    public static final double FWD_DISTANCE_2 = 50.0;
    public static final double FWD_DISTANCE_3 = 208.0;
    public static final double START_POS_1 = -102.0;
    public static final double START_POS_2 = 0.0;
    public static final double START_POS_3 = 102.0;
    
}   // class RobotInfo
