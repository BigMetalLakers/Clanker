## Quick orientation for AI contributors

This repository is a WPILib (2026) Java robot project for a swerve-drive robot.
Give concise, codebase-specific edits: prefer small, targeted changes that follow existing patterns.

Key places to read first:
- `src/main/java/frc/robot/RobotContainer.java` — sets up subsystems, controller bindings, and the PathPlanner auto chooser.
- `src/main/java/frc/robot/subsystems/DriveSubsystem.java` — swerve odometry, PhotonVision/AprilTag fusion, PathPlanner integration, default drive command.
- `src/main/java/frc/robot/subsystems/MAXSwerveModule.java` — motor/encoder configuration and per-module control.
- `src/main/java/frc/robot/subsystems/CANFuelSubsystem.java` — intake/launcher motor logic and SmartDashboard keys.
- `src/main/java/frc/robot/Configs.java` and `Constants.java` — hardware CAN IDs, kinematics, tuning constants.
- `build.gradle` — GradleRIO plugin configuration; deploy and simulation targets are configured here.

Architecture summary (big picture):
- Command-based robot: subsystems expose command factories (e.g. `intakeCommand()`, `spinUpCommand()`) and default commands are set in `RobotContainer`.
- Swerve drive: four `MAXSwerveModule` instances in `DriveSubsystem`. Kinematics and odometry use `DriveConstants.kDriveKinematics` and `SwerveDrivePoseEstimator`.
- Vision: PhotonCamera(s) (`TAG_CAMERA_NAME`, `FUEL_CAMERA_NAME` in `Constants`) feed AprilTag/target estimates into the pose estimator (`m_odometry.addVisionMeasurement`).
- Autos: PathPlanner autos are loaded from `src/main/deploy/pathplanner/*.auto` and registered via `AutoBuilder` / `PathPlannerAuto`.
- Hardware config: motor/controller configuration is centralized in `Configs.java` and applied in `MAXSwerveModule` and `CANFuelSubsystem`.

Project-specific conventions and patterns (do not change without cause):
- SmartDashboard keys are authoritative for tunable voltages and are used by subsystems (see `CANFuelSubsystem` e.g. `"INTAKING_FEEDER_VOLTAGE"`). If you add a tunable value, add a matching dashboard put/get usage.
- Subsystems expose small command factories using `this.run(...)` or `this.runEnd(...)` (see `CANFuelSubsystem` and `DriveSubsystem#setX()`); follow this pattern when creating new button/trigger actions.
- Module offsets: `MAXSwerveModule` expects a chassis angular offset passed from `Constants` — keep offsets in `Constants.ModuleConstants`/`DriveConstants`.
- PathPlanner integration: RobotConfig is loaded via `RobotConfig.fromGUISettings()` in `DriveSubsystem` (wrapped in try/catch). If modifying PathPlanner behavior, preserve the `AutoBuilder.configure(...)` call signatures.

Integrations and external dependencies to be aware of:
- WPILib (GradleRIO). Build with Gradle tasks (see below).
- REV Spark/Through-Bore encoder APIs (`com.revrobotics.*`) used in `MAXSwerveModule` and `Configs`.
- PhotonVision (org.photonvision.*) — used to get target poses and fuse into odometry in `DriveSubsystem`.
- PathPlanner (com.pathplanner.lib) — autos and PPHolonomicDriveController are configured in `DriveSubsystem` and `RobotContainer`.

Common developer workflows (concrete commands for Windows PowerShell):
- Compile: `./gradlew build` (Windows PowerShell: `.\gradlew.bat build` or `.\gradlew build`).
- Deploy to RoboRIO: `.\gradlew deploy` (team number must be in wpilib preferences or passed via `-Pteam=<num>`). See `deploy` block in `build.gradle`.
- Run unit tests: `.\gradlew test`.
- Run simulation (desktop): GradleRIO provides simulation tasks; common task is `.\gradlew simulate` (may open simulator GUI). If unsure, run `.\gradlew tasks --all`.

Patterns for modifying code safely:
- Small, isolated edits: change SmartDashboard key defaults in `CANFuelSubsystem` and Constants together.
- When changing control gains, update `Configs.java` and persist to hardware via `SparkMax.configure(...)` calls (see `MAXSwerveModule` constructor).
- For odometry/vision changes, preserve the existing use of `SwerveDrivePoseEstimator` and `m_odometry.addVisionMeasurement(...)` to avoid degrading pose fusion.

Example editing tasks and where to implement them:
- Add a new tunable voltage: add default in `Constants.FuelConstants`, put on SmartDashboard in `CANFuelSubsystem` constructor, read it where used (follow existing keys).
- Add a new controller binding: edit `RobotContainer.configureBindings()` and use the existing pattern of `m_operatorController.button().whileTrue(subsystem.commandFactory())`.
- Add a new PathPlanner auto: add `.auto` file under `src/main/deploy/pathplanner` and it will be auto-discovered by `RobotContainer`.

Testing and verification notes:
- The project is Java 17. GradleRIO plugin handles test configuration; run `.\gradlew test` to run JUnit 5 tests.
- Quick static checks: build (`.\gradlew build`) before pushing changes. Keep edits minimal to avoid long debug cycles.

Files worth scanning when troubleshooting:
- `DriveSubsystem.java` — gyro axis (`ADIS16470_IMU.IMUAxis.kZ`) and `getHeading()` conversions are a frequent source of sign/units bugs.
- `MAXSwerveModule.java` — encoder conversions, `optimize(...)` call, and closed-loop reference types (velocity vs position).
- `RobotContainer.java` — default commands and trigger wiring; ensures commands are scheduled with correct requirements.

If you need more context or want the README expanded, ask for specific areas (e.g., deploy notes, testing, or simulation setup) and include a short proposed change to the file.

Please review and tell me which sections need more detail or examples to help new contributors get productive faster.
