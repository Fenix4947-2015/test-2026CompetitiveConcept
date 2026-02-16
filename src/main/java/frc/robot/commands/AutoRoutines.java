// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;

import static frc.robot.generated.ChoreoTraj.choreo_1m_13_14;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class AutoRoutines {
    private final Swerve swerve;
    // Optional subsystems; may be null when running minimal path-planning tests.
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;
    
    private final SubsystemCommands subsystemCommands; // may be null
    
    private final List<String> availableAutos = new ArrayList<>();
    
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;
    
    public AutoRoutines(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight
        ) {
            this.swerve = swerve;
            this.intake = intake;
            this.floor = floor;
            this.feeder = feeder;
            this.shooter = shooter;
            this.hood = hood;
            this.hanger = hanger;
            this.limelight = limelight;
            
            this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);
            
            this.autoFactory = swerve.createAutoFactory();
            this.autoChooser = new AutoChooser();
    }
    
    /**
     * Minimal constructor for path-planning tests. Only swerve and limelight are required.
     */
    public AutoRoutines(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.intake = null;
        this.floor = null;
        this.feeder = null;
        this.shooter = null;
        this.hood = null;
        this.hanger = null;
        this.limelight = limelight;

        this.subsystemCommands = null;
        
        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    /**
     * Configure the AutoChooser with available routines. This should be called during RobotContainer initialization.
     */
    public void configure() {
        autoChooser.addRoutine("Choreo 1m 13-14", this::choreo_1m_13_14);
        availableAutos.add("Choreo 1m 13-14");
        
        // Discover PathPlanner autos and add them to the chooser/list so the dashboard shows one unified list
        // final Path ppAutos = Paths.get("/deploy/pathplanner/autos");
        final Path ppAutos = Paths.get("D:","Antoine","Documents","Code","FRC","2026","test-2026CompetitiveConcept","src","main","deploy","pathplanner","autos");
        if (Files.exists(ppAutos) && Files.isDirectory(ppAutos)) {
            try {
                final List<String> ppNames = Files.list(ppAutos)
                    .filter(Files::isRegularFile)
                    .map(Path::getFileName)
                    .map(Path::toString)
                    .filter(n -> n.endsWith(".auto") || n.endsWith(".path"))
                    .collect(Collectors.toList());

                for (String name : ppNames) {
                    // Add to available list and register a minimal routine so chooser can select it.
                    // The AutoFactory.newRoutine(name) creates a named routine; replace with actual PathPlanner
                    // loader if available in your AutoFactory implementation.
                    availableAutos.add(name);
                    autoChooser.addRoutine(name, () -> autoFactory.newRoutine(name));
                }
            } catch (IOException e) {
                SmartDashboard.putString("PathPlanner Autos Error", e.getMessage());
            }
        }

        // Publish a single combined list
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putStringArray("Available Autos", availableAutos.toArray(new String[0]));
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine choreo_1m_13_14() {
        final AutoRoutine routine = autoFactory.newRoutine("Choreo 1m 13-14");
        final AutoTrajectory loop13_14 = choreo_1m_13_14.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                loop13_14.resetOdometry(),
                loop13_14.cmd()
            )
        );

        return routine;
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                startToOutpost.cmd()
            )
        );

        if (hanger != null) {
            routine.observe(hanger::isHomed).onTrue(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    intake != null ? intake.runOnce(() -> intake.set(Intake.Position.INTAKE)) : Commands.none()
                )
            );
        }

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        if (intake != null) {
            outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        }
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        depotToShootingPose.active().whileTrue(limelight.idle());
        if (shooter != null && hood != null) {
            depotToShootingPose.atTime(0.5).onTrue(
                Commands.parallel(
                    shooter.spinUpCommand(2600),
                    hood.positionCommand(0.32)
                )
            );
        }
        if (subsystemCommands != null) {
            depotToShootingPose.done().onTrue(
                Commands.sequence(
                    subsystemCommands.aimAndShoot()
                        .withTimeout(5),
                    shootingPoseToTower.cmd()
                )
            );
        } else {
            // No subsystem commands available; just continue to next trajectory
            depotToShootingPose.done().onTrue(shootingPoseToTower.cmd());
        }

        shootingPoseToTower.active().whileTrue(limelight.idle());
        if (hanger != null) {
            shootingPoseToTower.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
            shootingPoseToTower.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
        }

        return routine;
    }

    // Expose the selected autonomous as a Command for Robot.autonomousInit() to schedule.
    public Command getSelected() {
        return autoChooser.selectedCommandScheduler();
    }
}
