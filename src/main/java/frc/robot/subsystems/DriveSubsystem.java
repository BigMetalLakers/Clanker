// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

//Gyro
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

// Photo Switch
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

// PhotonVision Libraries
//https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
import org.photonvision.PhotonCamera; // current version is v2025.1.1
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// Additional Robot Field and Pose Libriaries
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields; // this is marked for deprication, but works for now
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// Dashboard Libriaries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // delete this comment if this works with elastic
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.util.sendable.SendableRegistry;

//PathPlannerLibraries
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import org.json.simple.parser.ParseException;
import java.io.IOException;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      FRONT_LEFT_DRIVE_ID,
      FRONT_LEFT_TURN_ID,
      FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      FRONT_RIGHT_DRIVE_ID,
      FRONT_RIGHT_TURN_ID,
      FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      BACK_LEFT_DRIVE_ID,
      BACK_LEFT_TURN_ID,
      BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      BACK_RIGHT_DRIVE_ID,
      BACK_RIGHT_TURN_ID,
      BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // m_gyro.setRobotPose(DriveConstants.kHome2dPose);

  // Photo Switch
  private final DigitalInput PhotoSwitch = new DigitalInput(0);

  // Relative Drive Mode
  public Boolean fCentricBoolean = false;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry;

  // declare and instantiate the camera and pose objects
  // PhotonCamera tagCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  // PhotonCamera noteCamera = new PhotonCamera("AVerMedia_PW315");
  // Robot position (0,0) is the centre of the robot.
  PhotonCamera tagCamera = new PhotonCamera(DriveConstants.TAG_CAMERA_NAME);
  PhotonCamera fuelCamera = new PhotonCamera(DriveConstants.FUEL_CAMERA_NAME);
  public static boolean tagTarget;
  public static boolean pHasTarget, hasTagTarget, hasFuelTarget, weAreBlueAlliance;

  public static int tagTargetID;
  public Transform3d cameraToRobot = DriveConstants.TAG_CAMERA_TO_ROBOT;
  public Translation2d hubCentre;
  public static double YawErrorToShootAtHubCentre, YawErrorToFuel, DistanceToHub, visionVoltage;


  public Pose2d robotPosition = new Pose2d();
  private final Field2d m_field = new Field2d();
  public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();

  //PathPlanner
  private final RobotConfig PPconfig; 

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  
  //load PathPlanner configs from GUI
    try {
      PPconfig = RobotConfig.fromGUISettings();            // <-- canonical call
  } catch (IOException | ParseException e) {
      // handle the error (report it and provide a safe fallback)
      DriverStation.reportError("Failed to load PathPlanner GUI settings: " + e.getMessage(), e.getStackTrace());
      throw new RuntimeException("PathPlanner RobotConfig load failed", e);
      // or set config = createFallbackConfig();
  }

    // get our alliance colour
    var alliance = DriverStation.getAlliance();
    weAreBlueAlliance = (alliance.get() == DriverStation.Alliance.Blue);
    hubCentre = weAreBlueAlliance ? BLUE_HUB_CENTRE : RED_HUB_CENTRE;
    
    m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        Pose2d.kZero,        
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    robotPosition = DriveConstants.kHome2dPose;
    m_odometry.resetPose(robotPosition);

    SendableRegistry.add(m_gyro, "IMU");
    SendableRegistry.add(m_field, "Field");
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelativeSpeed(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      PPconfig, // The robot configuration
      () -> {
   
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        return !weAreBlueAlliance;
      },
      this // Reference to this subsystem to set requirements
    );
   

  }

  @Override
  public void periodic() {

    // Check the Camera for a Target
    var tagResult = tagCamera.getLatestResult();
    tagTarget = tagResult.hasTargets();
    if (tagTarget) {
      PhotonTrackedTarget target = tagResult.getBestTarget();
      tagTargetID = target.getFiducialId();
      Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagTargetID).get();
      Pose3d pvRobotPose3D = PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),
          tagPose,
          cameraToRobot);
      Pose2d pvRobotPose2D = new Pose2d(
          pvRobotPose3D.getTranslation().toTranslation2d(),
          pvRobotPose3D.getRotation().toRotation2d());
      m_odometry.addVisionMeasurement(pvRobotPose2D, tagResult.getTimestampSeconds());
    }
    // Update the odometry in the periodic block
    robotPosition = m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_field.setRobotPose(robotPosition);

// find angle to turn to make shooter face the centre of hub
    YawErrorToShootAtHubCentre = MathUtil.inputModulus(hubCentre.minus(robotPosition.getTranslation())
        .getAngle()
        .minus(robotPosition.getRotation())
        .getDegrees() + 180, -180, 180);

    DistanceToHub = getDistanceToHub(robotPosition, hubCentre);

    visionVoltage = getVoltageForShot(robotPosition, hubCentre);

    // find the angle to fuel
    PhotonPipelineResult fuelResult = fuelCamera.getLatestResult();
    if (fuelResult.hasTargets()) {
      hasFuelTarget = true;
      PhotonTrackedTarget fuelTarget = fuelResult.getBestTarget();
      YawErrorToFuel = MathUtil.inputModulus(fuelTarget.getYaw(), -180, 180); // degrees
    } else {
      hasFuelTarget = false;
    }


    SmartDashboard.putNumber("Yaw to Hub", YawErrorToShootAtHubCentre);
    SmartDashboard.putNumber("visionVoltage", visionVoltage);
    SmartDashboard.putNumber("Distance to Hub", DistanceToHub);
    SmartDashboard.putBoolean("Field Centric", fCentricBoolean);
    SmartDashboard.putBoolean("Photo Switch", PhotoSwitch.get());
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putBoolean("April Tag", tagTarget);
    SmartDashboard.putNumber("Target ID", tagTargetID);
    SmartDashboard.putData("Field", m_field);

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_MPS;
    double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_MPS;
    double rotDelivered = rot * DriveConstants.MAX_ANGULAR_SPEED_RPS;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command setX() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  /**
   * 
   * @param desiredStates
   */
  public Command alterFieldCentric(boolean state) {
    return this.runOnce(
        () -> fCentricBoolean = state);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the rotation effort value that replaces the x-box right stick x value
   * for turning to the robot 180° from the hub centre to align a shot.
   *
   * @return the rotation effort value that replaces the x-box right stick x value
   *         for turning to make a shot.
   */
  public double getShotTurn() {
    double error = YawErrorToShootAtHubCentre;
    double rot = TURN_PROPORTION * error;
    if (Math.abs(error) > ANGLE_TO_STOP_MINIMUM_TURN_EFFORT) {
      rot += MINIMUM_TURN_EFFORT * Math.signum(error);
    }
    return MathUtil.clamp(rot, -MAXIMUM_TURN_EFFORT,MAXIMUM_TURN_EFFORT);
  }

  /**
   * Returns the rotation effort value that replaces the x-box right stick x value
   * for turning towards fuel.
   *
   * @return the rotation effort value that replaces the x-box right stick x value
   *         for turning towards fuel
   */
  public double getFuelTurn() {
    double error = YawErrorToFuel;
    double rot = TURN_PROPORTION * error;
    if (Math.abs(error) > ANGLE_TO_STOP_MINIMUM_TURN_EFFORT) {
      rot += MINIMUM_TURN_EFFORT * Math.signum(error);
    }
    return MathUtil.clamp(rot, -MAXIMUM_TURN_EFFORT,MAXIMUM_TURN_EFFORT);
  }




  public double getDistanceToHub(Pose2d robotPosition, Translation2d hubCentre) {
    return robotPosition.getTranslation()
        .getDistance(hubCentre);
  }
  
  public double getVoltageForShot(Pose2d robotPosition, Translation2d hubCentre) {
    return (-1)*((robotPosition.getTranslation()
        .getDistance(hubCentre))*2.04 + 1.12);
        //should the visonVoltage go here?
 
    

   }
   
   // PathPlanner Methods

  /**
   * Method to drirve the robot using Chassis Speeds
   * 
   * @param speeds Chassis speeds of the robot
   */
  public void driveRobotRelativeSpeed(ChassisSpeeds speeds) {
    drive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        false);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * 
   * @return Current Chassis Speeds of the Robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

}
