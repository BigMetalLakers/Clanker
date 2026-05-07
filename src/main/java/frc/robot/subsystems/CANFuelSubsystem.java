// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

import java.util.function.Supplier;

public class CANFuelSubsystem extends SubsystemBase {
    private final SparkFlex feederRoller,intakeRoller,launchRoller;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeRoller = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkFlex(FEEDER_MOTOR_ID, MotorType.kBrushless);
    launchRoller = new SparkFlex(LAUNCH_MOTOR_ID, MotorType.kBrushless);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("INTAKING_FEEDER_VOLTAGE", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("LAUNCHING_FEEDER_VOLTAGE", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("LAUNCHING_LAUNCHER_VOLTAGE", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("LAUNCHING_LAUNCHER_VOLTAGE_SOFT", LAUNCHING_LAUNCHER_VOLTAGE_SOFT);
    SmartDashboard.putNumber("SPIN_UP_FEEDER_VOLTAGE", SPIN_UP_FEEDER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the intake roller, set a current limit, set
    // the motor to inverted so that positive values are used intakeing and
    // apply the config to the controller
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(INVERT_INTAKE_MOTOR);
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig launcherConfig = new SparkFlexConfig();
    launcherConfig.inverted(INVERT_LAUNCH_MOTOR);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launchRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("INTAKING_FEEDER_VOLTAGE", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(SmartDashboard.getNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE));
  }

  // A command factory to turn the intake method into a command that requires this
  // subsystem
  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("INTAKING_FEEDER_VOLTAGE", INTAKING_FEEDER_VOLTAGE));
    intakeRoller
        .setVoltage(-1 * SmartDashboard.getNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE));
  }

  // A command factory to turn the eject method into a command that requires this
  // subsystem
  public Command ejectCommand() {
    return this.run(() -> eject());
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber("LAUNCHING_FEEDER_VOLTAGE", LAUNCHING_FEEDER_VOLTAGE));
    intakeRoller
       .setVoltage(SmartDashboard.getNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE));
    launchRoller
        .setVoltage(SmartDashboard.getNumber("LAUNCHING_LAUNCHER_VOLTAGE", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void launchSoft() {
    feederRoller.setVoltage(SmartDashboard.getNumber("LAUNCHING_FEEDER_VOLTAGE", LAUNCHING_FEEDER_VOLTAGE));
    intakeRoller
       .setVoltage(SmartDashboard.getNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE));
    launchRoller
        .setVoltage(SmartDashboard.getNumber("LAUNCHING_LAUNCHER_VOLTAGE_SOFT", LAUNCHING_LAUNCHER_VOLTAGE_SOFT));
  }

  public void launchauto() {
    //fairly sure that this (below) is incorrect, there should not be a (-1) * ....
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("LAUNCHING_FEEDER_VOLTAGE", LAUNCHING_FEEDER_VOLTAGE));
    intakeRoller
       .setVoltage(SmartDashboard.getNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE));
    launchRoller
        .setVoltage(SmartDashboard.getNumber("LAUNCHING_LAUNCHER_VOLTAGE", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launchauto());
  }

  public Command launchCommandSoft() {
    return this.run(() -> launchSoft());
  }

    public Command launchautoCommand() {
    return this.run(() -> launch());
  }

  public void launchToDistance(
      DriveSubsystem m_DriveSubsystem,
      Pose2d robotPosition,
      Translation2d hubCentre) {

    double voltage = m_DriveSubsystem.getVoltageForShot(robotPosition, hubCentre);
    feederRoller.setVoltage(SmartDashboard.getNumber("LAUNCHING_FEEDER_VOLTAGE", LAUNCHING_FEEDER_VOLTAGE));
    intakeRoller.setVoltage((-1) * SmartDashboard.getNumber("INTAKING_INTAKE_VOLTAGE", INTAKING_INTAKE_VOLTAGE));
    launchRoller.setVoltage(voltage);
  }
  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchToDistanceCommand(
    DriveSubsystem m_robotDrive,
    Supplier<Pose2d> robotPoseSupplier,
    Supplier<Translation2d> hubCentreSupplier
  ) {
    return this.run(() -> launchToDistance(
      m_robotDrive,
      robotPoseSupplier.get(),
      hubCentreSupplier.get()
    ));
  }
  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeRoller.set(0);
    launchRoller.set(0);
  }

  // A command factory to turn the eject method into a command that requires this
  // subsystem
  public Command stopRollersCommand() {
    return this.run(() -> stop());
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("SPIN_UP_FEEDER_VOLTAGE", SPIN_UP_FEEDER_VOLTAGE));
    launchRoller
        .setVoltage(SmartDashboard.getNumber("LAUNCHING_LAUNCHER_VOLTAGE", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
