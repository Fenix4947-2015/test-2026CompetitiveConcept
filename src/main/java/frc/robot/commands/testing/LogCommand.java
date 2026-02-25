package frc.robot.commands.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.util.RepeatabilityLogger;

public class LogCommand extends Command {

    private final Swerve swerve;
    private final RepeatabilityLogger logger;
    private final Translation2d goal;

    private boolean hasLogged = false;

    public LogCommand(Swerve swerve,
                      RepeatabilityLogger logger,
                      Translation2d goal) {

        this.swerve = swerve;
        this.logger = logger;
        this.goal = goal;
    }

    @Override
    public void execute() {

        if (hasLogged) return;

        final Pose2d pose = swerve.getState().Pose;

        double distanceToGoal =
                pose.getTranslation()
                        .getDistance(goal);

        Rotation2d desiredAngle = goal
                        .minus(pose.getTranslation())
                        .getAngle();

        double angleErrorDeg =
                desiredAngle.minus(pose.getRotation()).getDegrees();

        logger.log(
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees(),
                distanceToGoal,
                angleErrorDeg
        );

        hasLogged = true;
    }

    @Override
    public boolean isFinished() {
        return hasLogged;
    }
}