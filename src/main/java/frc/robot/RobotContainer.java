// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.HIGH_RIGHT_THROTTLE_LEVEL;
import static frc.robot.Constants.OperatorConstants.LOW_LEFT_THROTTLE_LEVEL;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CANFuelSubsystem m_fuelSubsystem = new CANFuelSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    NamedCommands.registerCommand("Launch", (m_fuelSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(m_fuelSubsystem.launchCommand())
            .finallyDo(() -> m_fuelSubsystem.stop())));
    // NamedCommands.registerCommand("LaunchToDistance",(m_fuelSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
    //         .andThen(m_fuelSubsystem.launchToDistanceCommand(
    //           m_robotDrive,
    //           m_robotDrive::getPose,
    //           m_robotDrive.hubCentre
    //         ))
    //         .finallyDo(() -> m_fuelSubsystem.stop())));
    NamedCommands.registerCommand("SpinUp", m_fuelSubsystem.spinUpCommand());
    NamedCommands.registerCommand("Intake", m_fuelSubsystem.intakeCommand());
    NamedCommands.registerCommand("Eject", m_fuelSubsystem.ejectCommand());
    NamedCommands.registerCommand("StopRollers", m_fuelSubsystem.stopRollersCommand());
    configureBindings();
    configureAutoChooser();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    m_operatorController.leftBumper()
        .whileTrue(m_fuelSubsystem.runEnd(() -> m_fuelSubsystem.intake(), () -> m_fuelSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
   
    m_operatorController.rightBumper()
        .whileTrue(m_fuelSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(m_fuelSubsystem.launchCommand())
            .finallyDo(() -> m_fuelSubsystem.stop()));
    // m_operatorController.b()
    //     .whileTrue(m_fuelSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
    //         .andThen(m_fuelSubsystem.launchToDistanceCommand())
     //       .finallyDo(() -> m_fuelSubsystem.stop()));
            
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    m_operatorController.a()
        .whileTrue(m_fuelSubsystem.runEnd(() -> m_fuelSubsystem.eject(), () -> m_fuelSubsystem.stop()));

    //Soft launch with low voltage
    m_operatorController.y()
        .whileTrue(m_fuelSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(m_fuelSubsystem.launchCommandSoft())
            .finallyDo(() -> m_fuelSubsystem.stop()));

    m_driverController.x().whileTrue(m_robotDrive.setX());

    m_driverController.rightBumper().onTrue(m_robotDrive.alterFieldCentric(true));
    m_driverController.leftBumper().onTrue(m_robotDrive.alterFieldCentric(false));

    m_robotDrive.setDefaultCommand(
    new RunCommand(() -> {
        // Forward/X motion
        double xSpeed = -m_driverController.getLeftY() *
            (m_driverController.getLeftTriggerAxis() * LOW_LEFT_THROTTLE_LEVEL
             + m_driverController.getRightTriggerAxis() * HIGH_RIGHT_THROTTLE_LEVEL);
        // Left/Y strafe motion
        double ySpeed = -m_driverController.getLeftX() *
            (m_driverController.getLeftTriggerAxis() * LOW_LEFT_THROTTLE_LEVEL
             + m_driverController.getRightTriggerAxis() * HIGH_RIGHT_THROTTLE_LEVEL);
        // Rotation/Yaw motion
        double rot;
        if (m_driverController.a().getAsBoolean()) {
            // Aim at hub
            rot = m_robotDrive.getShotTurn();
        } else if (m_driverController.b().getAsBoolean()) {
            // Aim at fuel
            rot = m_robotDrive.getFuelTurn();
        } else {
            // Manual rotation
            rot = -m_driverController.getRightX() *
                (m_driverController.getLeftTriggerAxis() * LOW_LEFT_THROTTLE_LEVEL
                 + m_driverController.getRightTriggerAxis() * HIGH_RIGHT_THROTTLE_LEVEL);
        }

        // Call drive
        m_robotDrive.drive(xSpeed, ySpeed, rot, m_robotDrive.fCentricBoolean);

    }, m_robotDrive) // Subsystem requirement
);
   
  

  }
   
  

  private void configureAutoChooser() {

    // --- If using PathPlanner Autos (recommended) ---
    // This loads all .auto files automatically and creates commands
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    if (autoNames.isEmpty()) {
      System.out.println("⚠ No PathPlanner autos found!");
    }

    boolean first = true;
    for (String name : autoNames) {
      Command auto = new PathPlannerAuto(name);
      if (first) {
        autoChooser.setDefaultOption(name, auto);
        first = false;
      } else {
        autoChooser.addOption(name, auto);
      }
    }

    // Publish to dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

}
