// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    // Key Locations 
    public static final Pose2d kHome2dPose = new Pose2d(12.31,1.86, new Rotation2d(Units.degreesToRadians(-60)));
    // public static final int LEFT_LEADER_ID = 1;
    // public static final int LEFT_FOLLOWER_ID = 2;
    // public static final int RIGHT_LEADER_ID = 3;
    // public static final int RIGHT_FOLLOWER_ID = 4;

    public static final double MAX_SPEED_MPS = 4.8;
    public static final double MAX_ANGULAR_SPEED_RPS = 2 * Math.PI;

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVE_ID = 11;
    public static final int BACK_LEFT_DRIVE_ID = 13;
    public static final int FRONT_RIGHT_DRIVE_ID = 15;
    public static final int BACK_RIGHT_DRIVE_ID = 17;

    public static final int FRONT_LEFT_TURN_ID = 10;
    public static final int BACK_LEFT_TURN_ID = 12;
    public static final int FRONT_RIGHT_TURN_ID = 14;
    public static final int BACK_RIGHT_TURN_ID = 16;

    public static final boolean GYRO_REVERSED = false;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

     // Camera Constants
    public static final Transform3d TAG_CAMERA_TO_ROBOT = new Transform3d(0.3, 0.11, -0.19, new Rotation3d
       (0, Units.degreesToRadians(-44.13), Units.degreesToRadians(180))); 
    public static final String TAG_CAMERA_NAME ="Arducam_OV9281_USB_Camera";
    public static final String FUEL_CAMERA_NAME ="AVerMedia_PW315";

    // target translations
    public static final Translation2d BLUE_HUB_CENTRE = new Translation2d(((158.34 + 44.2) * (2.54 / 100)),
        (158.84) * (2.54 / 100));
    public static final Translation2d RED_HUB_CENTRE = new Translation2d(((492.88 - 44.2) * (2.54 / 100)),
        (158.84) * (2.54 / 100));

    public static final double MINIMUM_TURN_EFFORT = 0.080;
    public static final double TURN_PROPORTION = 0.020;
    public static final double ANGLE_TO_STOP_MINIMUM_TURN_EFFORT = 1.5;
    public static final double MAXIMUM_TURN_EFFORT = 0.600;
    

    

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_RUNNING_FREQUENCY = NeoMotorConstants.FREE_RUNNING_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_RUNNING_FREQUENCY = (DRIVING_MOTOR_FREE_RUNNING_FREQUENCY * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;
  }

  public static final class NeoMotorConstants {
    public static final double FREE_RUNNING_RPM = 5676;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 4;
    public static final int INTAKE_MOTOR_ID = 5;
    public static final int LAUNCH_MOTOR_ID = 6;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    public static final boolean INVERT_FEEDER_MOTOR = true;
    public static final boolean INVERT_INTAKE_MOTOR = true;
    public static final boolean INVERT_LAUNCH_MOTOR = true;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -4.5;
    public static final double INTAKING_INTAKE_VOLTAGE = -4.5;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 8;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = -7;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE_SOFT = -5;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 2;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double LOW_LEFT_THROTTLE_LEVEL = .3;
    public static final double HIGH_RIGHT_THROTTLE_LEVEL = 1 - LOW_LEFT_THROTTLE_LEVEL;
  }
}
