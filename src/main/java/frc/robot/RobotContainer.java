// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
// Disabled subsystems for path-planning tests: Intake, Floor, Feeder, Shooter, Hood, Hanger
// They are intentionally not instantiated below so autos and swerve/limelight testing remain simple.
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.VisionMeasurement;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    // private final Intake intake = new Intake();
    // private final Floor floor = new Floor();
    // private final Feeder feeder = new Feeder();
    // private final Shooter shooter = new Shooter();
    // private final Hood hood = new Hood();
    // private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Autos */
    private final SendableChooser<Command> autoChooser;
    public double auto_delay = 0.0;

    // Semi-Autos
    private enum GoTrench {
        GO_LEFT_CENTER,
        GO_RIGHT_CENTER,
        GO_LEFT_GOAL,
        GO_RIGHT_GOAL,
        NONE,
    }

    // Vision updates
    private int consistentFrames = 0;
    private static final int REQUIRED_FRAMES = 3; // Number of consecutive frames with a valid measurement required to consider the pose estimate reliable.
    private boolean needsReset = false;
    private boolean firstBoot = true;

    // Create AutoRoutines with only the subsystems we want enabled for path-planning tests.
    private final AutoRoutines autoRoutines = new AutoRoutines(swerve, limelight);
    // SubsystemCommands (and the subsystems they depend on) are disabled for this test run.
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        null,
        null,
        null,
        null,
        null,
        null,
        () -> -driver.getLeftY(), 
        () -> -driver.getLeftX()
        );

    // Networktables publishers
    private final NetworkTable telemetryTable;
    private final IntegerPublisher visionRawEmptyCountPublisher;
    private final IntegerPublisher visionFilterOutlierEmptyCountPublisher;
    private final IntegerPublisher visionFilterEmptyCountPublisher;
    private final IntegerPublisher visionReseedCountPublisher;
    private final IntegerPublisher visionValidCountPublisher;
    private final StructPublisher<Pose2d> visionPosePublisher;
    private final DoubleArrayPublisher stdPosePublisher;
    private int visionRawEmptyCount = 0;  // Counter for how many consecutive frames the raw vision measurement has been empty.
    private int visionFilterOutlierEmptyCount = 0;
    private int visionFilterEmptyCount = 0;
    private int visionReseedCount = 0;
    private int visionValidCount = 0;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoChooser = buildAutoChooser();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);

        telemetryTable = NetworkTableInstance.getDefault().getTable("Vision");
        visionRawEmptyCountPublisher = telemetryTable.getIntegerTopic("Raw Empty Count").publish();
        visionFilterOutlierEmptyCountPublisher = telemetryTable.getIntegerTopic("Filter Outlier Empty Count").publish();
        visionFilterEmptyCountPublisher = telemetryTable.getIntegerTopic("Filter Empty Count").publish();
        visionReseedCountPublisher = telemetryTable.getIntegerTopic("Reseed Count").publish();
        visionValidCountPublisher = telemetryTable.getIntegerTopic("Valid Count").publish();
        visionPosePublisher = telemetryTable.getStructTopic("Estimated Vision Pose", Pose2d.struct).publish();
        stdPosePublisher = telemetryTable.getDoubleArrayTopic("Std dev Vision Pose").publish();
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        // Subsystems (intake, hanger, etc.) are commented out for path-planning tests.
        // If you want to re-enable operator-driven subsystem commands, uncomment and
        // restore the corresponding subsystem instances above.
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
        driver.povRight().onTrue(Commands.runOnce(() -> needsReset = true));  // Temporary: use the right button to trigger a vision reset when needed, until we have a more robust auto-reseeding strategy.
        // driver.rightTrigger().whileTrue(subsystemCommands.aim());   // tester si cette ligne fonctionne ou la suivante pour enligner le robot vers le but
        driver.rightTrigger().whileTrue(Commands.run(() -> manualDriveCommand.setLockedHeading(Landmarks.getDirectionToShoot(swerve))));

        try {
            // Semi-auto path
            driver.leftTrigger().whileTrue(
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("Semi_Auto"),
                    new PathConstraints(2, 1, 2, 1)
                    )
            );
            //TODO tester ça aussi:
            driver.rightBumper().whileTrue(
                AutoBuilder.followPath(
                    PathPlannerPath.fromPathFile("Semi_Auto")
                    )
            );
            //TODO You should never allow a path to start if pose estimate is garbage.
            // Commands.either(
            //     scorePath,
            //     Commands.print("Pose not valid"),
            //     () -> swerve.isPoseReliable()
            // );
        }
        catch (Exception e) {
            System.out.println("Error loading path: " + e.getMessage());
        }

        PathConstraints pathConstraints = new PathConstraints(2,2,2,2);
        LinearVelocity trenchSemiAuto = MetersPerSecond.of(1);

        //TODO utiliser la version flipped
        Command goThrenchCenterRedLeft = AutoBuilder.pathfindToPose(
            new Pose2d(10, 0.6, Rotation2d.fromDegrees(90)),   // Rouge gauche
            pathConstraints,
            trenchSemiAuto
        );
        Command goThrenchCenterRedRight = AutoBuilder.pathfindToPose(
            new Pose2d(10, 7.4, Rotation2d.fromDegrees(-90)),  // Rouge droite
            pathConstraints,
            trenchSemiAuto
            );
        Command goThrenchGoalRedLeft = AutoBuilder.pathfindToPose(
            new Pose2d(14, 1.635, Rotation2d.fromDegrees(130)),   // Rouge gauche
            pathConstraints,
            trenchSemiAuto
        );
        Command goThrenchGoalRedRight = AutoBuilder.pathfindToPose(
            new Pose2d(14, 6.365, Rotation2d.fromDegrees(50)),   // Rouge droite
            pathConstraints,
            trenchSemiAuto
        );

        Map<GoTrench,Command> goTrenchCommandMap = Map.of(
            GoTrench.GO_LEFT_CENTER, goThrenchCenterRedLeft,
            GoTrench.GO_RIGHT_CENTER, goThrenchCenterRedRight,
            GoTrench.GO_LEFT_GOAL, goThrenchGoalRedLeft,
            GoTrench.GO_RIGHT_GOAL, goThrenchGoalRedRight,
            GoTrench.NONE, Commands.none()
        );

        Supplier<GoTrench> goTrenchSelector = () -> {
            Pose2d p = swerve.getState().Pose;
            if (p.getX() > 12) {  // in goal zone, go to center
                if (p.getY() < 4) {
                    return GoTrench.GO_LEFT_CENTER;
                } else {
                    return GoTrench.GO_RIGHT_CENTER;
                }
            } else {  // in center zone, go to goal zone
                if (p.getY() < 4) {
                    return GoTrench.GO_LEFT_GOAL;
                } else {
                    return GoTrench.GO_RIGHT_GOAL;
                }
            }
        };

        Command enableTrenchSemiAuto = Commands.select(goTrenchCommandMap, goTrenchSelector);
        driver.leftBumper().whileTrue(enableTrenchSemiAuto);



        // //TODO tester move à position sans path find to pose
        // PathConstraints pathConstraints2 = new PathConstraints(2,2,2,2);
        // LinearVelocity trenchSemiAuto2 = MetersPerSecond.of(1);

        // //TODO utiliser la version flipped
        // Command goThrenchCenterRedLeft2 = AutoBuilder.pathfindToPose(
        //     new Pose2d(10, 0.6, Rotation2d.fromDegrees(90)),   // Rouge gauche
        //     pathConstraints2,
        //     trenchSemiAuto2
        // );
        // Command goThrenchCenterRedRight2 = AutoBuilder.pathfindToPose(
        //     new Pose2d(10, 7.4, Rotation2d.fromDegrees(-90)),  // Rouge droite
        //     pathConstraints2,
        //     trenchSemiAuto2
        //     );
        // Command goThrenchGoalRedLeft2 = AutoBuilder.pathfindToPose(
        //     new Pose2d(14, 1.635, Rotation2d.fromDegrees(130)),   // Rouge gauche
        //     pathConstraints2,
        //     trenchSemiAuto2
        // );
        // Command goThrenchGoalRedRight2 = AutoBuilder.pathfindToPose(
        //     new Pose2d(14, 6.365, Rotation2d.fromDegrees(50)),   // Rouge droite
        //     pathConstraints2,
        //     trenchSemiAuto2
        // );

        // Map<GoTrench,Command> goTrenchCommandMap2 = Map.of(
        //     GoTrench.GO_LEFT_CENTER, goThrenchCenterRedLeft2,
        //     GoTrench.GO_RIGHT_CENTER, goThrenchCenterRedRight2,
        //     GoTrench.GO_LEFT_GOAL, goThrenchGoalRedLeft2,
        //     GoTrench.GO_RIGHT_GOAL, goThrenchGoalRedRight2,
        //     GoTrench.NONE, Commands.none()
        // );

        // Supplier<GoTrench> goTrenchSelector2 = () -> {
        //     Pose2d p = swerve.getState().Pose;
        //     if (p.getX() > 12) {  // in goal zone, go to center
        //         if (p.getY() < 4) {
        //             return GoTrench.GO_LEFT_CENTER;
        //         } else {
        //             return GoTrench.GO_RIGHT_CENTER;
        //         }
        //     } else {  // in center zone, go to goal zone
        //         if (p.getY() < 4) {
        //             return GoTrench.GO_LEFT_GOAL;
        //         } else {
        //             return GoTrench.GO_RIGHT_GOAL;
        //         }
        //     }
        // };

        // Command enableTrenchSemiAuto2 = Commands.select(goTrenchCommandMap2, goTrenchSelector2);
        // driver.rightBumper().whileTrue(enableTrenchSemiAuto2);

    }



    // private Command updateVisionCommand() {
    //     return limelight.run(() -> {
    //         final Pose2d currentRobotPose = swerve.getState().Pose;
    //         final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
    //         measurement.ifPresent(m -> {

    //             visionPosePublisher.set(m.poseEstimate.pose);

    //             swerve.addVisionMeasurement(
    //                 m.poseEstimate.pose, 
    //                 m.poseEstimate.timestampSeconds,
    //                 m.standardDeviations
    //             );
    //         });
    //     })
    //     .ignoringDisable(true);
    // }


    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final SwerveDriveState currentRobotState = swerve.getState();
            processVision(currentRobotState.Pose, currentRobotState.Speeds);
        })
        .ignoringDisable(true);
    }

    private void processVision(
            Pose2d currentPose,
            ChassisSpeeds speeds) {

        Optional<VisionMeasurement> raw =
            limelight.getRawMeasurement(currentPose);

        if (raw.isEmpty()) {
            consistentFrames = 0;
            visionRawEmptyCount++;
            return;
        }
        
        Optional<VisionMeasurement> filteredOutlier = limelight.filterOutlierMeasurement(raw.get(), currentPose);
        if (filteredOutlier.isEmpty()) {
            consistentFrames = 0;
            visionFilterOutlierEmptyCount++;
            return;
        }

        VisionMeasurement m = filteredOutlier.get();
        double translationError =
            m.pose.getTranslation()
                .getDistance(currentPose.getTranslation());

        // ----------------------------
        // Reseed condition in case we get lost or when we start the robot.
        // ----------------------------
        if ((needsReset && m.tagCount >= 2) ||  // manuel reseed
            (translationError > 3.0 && m.tagCount >= 2 && m.avgAmbiguity < 0.5) || // Auto reseed
            DriverStation.isDisabled() && m.tagCount >= 1  // Always reseed when disabled   RobotModeTriggers.disabled()
        ) {
        // if ((firstBoot && m.tagCount >=1) || (needsReset && m.tagCount >= 2)) {
            swerve.resetPose(m.pose);
            consistentFrames = 0;
            visionReseedCount++;
            needsReset = false;
            firstBoot = false;
            return;
        }

        Optional<VisionMeasurement> filtered =
            limelight.filterMeasurement(filteredOutlier.get(), currentPose);

        if (filtered.isEmpty()) {
            consistentFrames = 0;
            visionFilterEmptyCount++;
            return;
        }

        // Only update the pose estimator if we have a consistent stream of vision measurements.
        consistentFrames++;

        if (consistentFrames < REQUIRED_FRAMES)
            return;

        Matrix<N3,N1> stdDevs =
            // limelight.computeStdDevs(filtered.get(), speeds);
            VecBuilder.fill(0.1, 0.1, 10.0);

        // swerve.addVisionMeasurement(
        //     filtered.get().pose,
        //     filtered.get().timestamp,
        //     stdDevs
        // );
        swerve.addVisionMeasurement(
            filtered.get().pose,
            filtered.get().timestampSeconds,
            stdDevs
        );

        visionPosePublisher.set(filtered.get().pose);
        stdPosePublisher.set(stdDevs.getData());
        visionValidCount++;
    }

    private void processVision2(
            Pose2d currentPose,
            ChassisSpeeds speeds) {

        Optional<VisionMeasurement> measurement = getMeasurementFrom(limelight, currentPose);
        if (measurement.isEmpty()) {
            measurement = getMeasurementFrom(limelight, currentPose); //TODO utiliser limelight3
        }
        
        if (measurement.isEmpty()) {
            consistentFrames = 0;
            visionRawEmptyCount++;
            return;
        }

        // double translationError =
        //     m.pose.getTranslation()
        //         .getDistance(currentPose.getTranslation());

        // ----------------------------
        // Reseed condition in case we get lost or when we start the robot.
        // ----------------------------
        // if (needsReset || (translationError > 3.0 && m.tagCount >= 2)) {
        if (needsReset && measurement.get().tagCount >= 1) {
            swerve.resetPose(measurement.get().pose);
            consistentFrames = 0;
            visionReseedCount++;
            needsReset = false;
            return;
        }

        Optional<VisionMeasurement> filtered =
            limelight.filterMeasurement(measurement.get(), currentPose);

        if (filtered.isEmpty()) {
            consistentFrames = 0;
            visionFilterEmptyCount++;
            return;
        }

        // Only update the pose estimator if we have a consistent stream of vision measurements.
        consistentFrames++;

        if (consistentFrames < REQUIRED_FRAMES)
            return;

        Matrix<N3,N1> stdDevs =
            // limelight.computeStdDevs(filtered.get(), speeds);
            VecBuilder.fill(0.1, 0.1, 10.0);

        // swerve.addVisionMeasurement(
        //     filtered.get().pose,
        //     filtered.get().timestamp,
        //     stdDevs
        // );
        swerve.addVisionMeasurement(
            filtered.get().pose,
            filtered.get().timestampSeconds,
            stdDevs
        );

        visionPosePublisher.set(filtered.get().pose);
        stdPosePublisher.set(stdDevs.getData());
        visionValidCount++;
    }

    private Optional<VisionMeasurement> getMeasurementFrom(Limelight limelight, Pose2d currentPose) {
        Optional<VisionMeasurement> raw =
            limelight.getRawMeasurement(currentPose);

        if (raw.isEmpty()) {
            return Optional.empty();
        }
        
        Optional<VisionMeasurement> filteredOutlier = limelight.filterOutlierMeasurement(raw.get(), currentPose);
        if (filteredOutlier.isEmpty()) {
            return Optional.empty();
        }

        return filteredOutlier;
    }

    // Example: Cloned pose estimator for validation
    // private void clonedEstimator(){


    //     // Assume you have access to these from your main swerve
    //     SwerveDriveKinematics kinematics = swerve.getKinematics();
    //     SwerveModulePosition[] modulePositions = swerve.getState().ModulePositions;
    //     Rotation2d gyroAngle = swerve.getRotation3d().getRotation2d();
    //     Pose2d initialPose = swerve.getState().Pose;

    //     // Standard deviations for state and vision
    //     Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    //     Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.5, 0.5, 10.0);

    //     // Create the cloned estimator
    //     SwerveDrivePoseEstimator clonedEstimator = new SwerveDrivePoseEstimator(
    //         kinematics,
    //         gyroAngle,
    //         modulePositions,
    //         initialPose,
    //         stateStdDevs,
    //         visionStdDevs
    //     );

    //     // Periodically update with odometry
    //     clonedEstimator.update(gyroAngle, modulePositions);

    //     // When you get a vision measurement:
    //     Pose2d visionPose = ...; // from Limelight or other vision source
    //     double visionTimestamp = ...; // timestamp in seconds
    //     clonedEstimator.addVisionMeasurement(visionPose, visionTimestamp);

    //     // After several good vision updates, check if clonedEstimator.getEstimatedPosition() is stable
    //     // If so, you can reset your main estimator to this pose
    // }

    private SendableChooser<Command> buildAutoChooser() {
        final SendableChooser<Command> autoChooser;
        //TODO ajouter un délais variable avant chaque auto

        NamedCommands.registerCommand("Aim and Shoot", subsystemCommands.aim());
        // NamedCommands.registerCommand("toggle side gripper",new InstantCommand(m_coralGripper::toggleSideGripper, m_coralGripper));
        // NamedCommands.registerCommand("auto dunk coral right", autoDropCoralRight);
        // NamedCommands.registerCommand("auto dunk coral left", autoDropCoralLeft);
        // NamedCommands.registerCommand("auto get coral station 1",autoPickupCoralStation1);
        // NamedCommands.registerCommand("Arm L4",m_moveArmL4);
        // NamedCommands.registerCommand("Arm Lowest",m_moveArmLow);
        // NamedCommands.registerCommand("Grip Coral",gripCoral);
        // NamedCommands.registerCommand("Auto Delay",new WaitSmartDashBoard(smartDashboardSettings));
        // NamedCommands.registerCommand("Set Mega Tag",new SetMegaTag(limelightFour, drivetrain));
        // NamedCommands.registerCommand("Set Mega Tag 2",new SetMegaTag2(limelightFour, drivetrain));
        // NamedCommands.registerCommand("Set No Camera",new SetNoCamera(limelightFour, drivetrain));
        autoChooser = AutoBuilder.buildAutoChooser("auto_path");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putNumber("Auto Delay", auto_delay);
        return autoChooser;
    }

    /**
     * Return the currently selected autonomous command.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Get the current robot pose in meters, as estimated by odometry and vision. This is used for
     * visualizing the robot's position on the Field2d widget on the dashboard, 
     * and can also be used by commands that need to know the robot's current position.
     * @return the current robot pose in meters, as estimated by odometry and vision fusion.
     */
    public Pose2d getPoseMeters() {
        return swerve.getState().Pose;
    }

    public void Periodic() {
        // Update SmartDashboard with vision telemetry.
        visionRawEmptyCountPublisher.set(visionRawEmptyCount);
        visionFilterOutlierEmptyCountPublisher.set(visionFilterOutlierEmptyCount);
        visionFilterEmptyCountPublisher.set(visionFilterEmptyCount);
        visionReseedCountPublisher.set(visionReseedCount);
        visionValidCountPublisher.set(visionValidCount);
    }
}
