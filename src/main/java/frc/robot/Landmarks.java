package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

public class Landmarks {
    public static final Distance FIELD_LENGTH = Inches.of(651.22);
    public static final Distance FIELD_WIDTH = Inches.of(317.69);
    public static final Distance HUB_BLUE_X = Inches.of(182.105);
    public static final Distance HUB_Y = Inches.of(158.845);
    public static final Distance HOME_BLUE_X = Meters.of(3);
    public static final Distance HOME1_Y = Meters.of(1.5);
    public static final Distance HOME2_Y = FIELD_WIDTH.minus(HOME1_Y);

    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(HUB_BLUE_X, HUB_Y);
        }
        return new Translation2d(FIELD_LENGTH.minus(HUB_BLUE_X), HUB_Y);
    }

    public static Translation2d home1() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(HOME_BLUE_X, HOME1_Y);
        }
        return new Translation2d(FIELD_LENGTH.minus(HOME_BLUE_X), HOME1_Y);
    }

    public static Translation2d home2() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(HOME_BLUE_X, HOME2_Y);
        }
        return new Translation2d(FIELD_LENGTH.minus(HOME_BLUE_X), HOME2_Y);
    }

    public static Rotation2d getDirectionToHub(Swerve swerve) {
        final Translation2d hubPosition = hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        final Rotation2d hubDirectionInCurrentAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInCurrentAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }

    public static Rotation2d getDirectionToShoot(Swerve swerve) {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        final Translation2d hubPosition = hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        Translation2d shootTarget;

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            // Too close to line up with the hub, just use the robot's current heading to avoid erratic behavior.
            if (Math.abs(robotPosition.getX() - hubPosition.getX()) < Meters.of(0.3).in(Meters)) {
                return swerve.getState().Pose.getRotation();
            }
            // At home so we shoot towards the hub.
            if (robotPosition.getX() < hubPosition.getX()) {
                shootTarget = hubPosition;
            } else {
                // In the center so we shoot at home.
                if (robotPosition.getY() < hubPosition.getY()) {
                    shootTarget = home1();
                } else {
                    shootTarget = home2();
                }
            }
            
        } else {
            // Too close to line up with the hub, just use the robot's current heading to avoid erratic behavior.
            if (Math.abs(robotPosition.getX() - hubPosition.getX()) < Meters.of(0.3).in(Meters)) {
                return swerve.getState().Pose.getRotation();
            }
            // At home so we shoot towards the hub.
            if (robotPosition.getX() > hubPosition.getX()) {
                shootTarget = hubPosition;
            } else {
                // In the center so we shoot at home.
                if (robotPosition.getY() < hubPosition.getY()) {
                    shootTarget = home1();
                } else {
                    shootTarget = home2();
                }
            }
        }
        
        final Rotation2d hubDirectionInCurrentAlliancePerspective = shootTarget.minus(robotPosition).getAngle();
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInCurrentAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return hubDirectionInOperatorPerspective;
    }
}
