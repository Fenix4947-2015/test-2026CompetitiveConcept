package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Feeder.Speed;

public class Limelight extends SubsystemBase {
    private final String name;
    // private final NetworkTable telemetryTable;
    // private final StructPublisher<Pose2d> posePublisher;

    public Limelight(String name) {
        this.name = name;
        // this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        // this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        // posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    // public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
    //     LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    //     final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    //     if (
    //             poseEstimate_MegaTag2 == null
    //             || poseEstimate_MegaTag2.tagCount == 0
    //     ) {
    //         return Optional.empty();
    //     }

    //     final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.2, 0.2, 1.0);

    //     // posePublisher.set(poseEstimate_MegaTag2.pose);

    //     return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    // }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }

    public Optional<VisionMeasurement> getRawMeasurement(Pose2d robotPose) {

        LimelightHelpers.SetRobotOrientation(
            name,
            robotPose.getRotation().getDegrees(),
            0,0,0,0,0
        );

        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (mt2 == null 
            || mt2.tagCount == 0
            || mt1 == null
            || mt1.tagCount == 0) {
            return Optional.empty();
        }

        double ambig = mt2.rawFiducials[0].ambiguity;

        Pose2d pose = new Pose2d(
            mt2.pose.getTranslation(),
            mt1.pose.getRotation()
        );

        double timestamp =
            Timer.getFPGATimestamp()
            - mt2.latency / 1000.0;

        return Optional.of(
            new VisionMeasurement(
                pose,
                timestamp,
                mt2.avgTagDist,
                mt2.tagCount,
                mt2.timestampSeconds
            )
        );
    }

    public Optional<VisionMeasurement> filterOutlierMeasurement(VisionMeasurement m, Pose2d currentPose) {

        if (m.tagCount == 0)
            return Optional.empty();

        if (m.tagDistance > 5.5)
            return Optional.empty();

        if (!isInsideField(m.pose))
            return Optional.empty();

        return Optional.of(m);
    }

    public Optional<VisionMeasurement> filterMeasurement(VisionMeasurement m, Pose2d currentPose) {

        double translationError =
            m.pose.getTranslation()
                .getDistance(currentPose.getTranslation());

        double rotationError =
            Math.abs(
                m.pose.getRotation()
                    .minus(currentPose.getRotation())
                    .getRadians()
            );

        if (translationError > 2)
            return Optional.empty();

        if (rotationError > Math.toRadians(90))
            return Optional.empty();

        return Optional.of(m);
    }

    private boolean isInsideField(Pose2d pose) {

        double x = pose.getX();
        double y = pose.getY();

        return x > -0.5
            && x < 17.0
            && y > -0.5
            && y < 8.5;
    }

    public Matrix<N3, N1> computeStdDevs(VisionMeasurement m, ChassisSpeeds speeds) {

        double speed =
            Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
            );

        double omega =
            Math.abs(speeds.omegaRadiansPerSecond);

        double xy =
            0.1
            + m.tagDistance * 0.1
            + speed * 0.1
            + omega * 0.1;

        double theta =
            0.3
            + m.tagDistance * 0.1
            + omega * 0.1;

        return VecBuilder.fill(xy, xy, theta);
    }    

    public class VisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;
        public final double tagDistance;
        public final int tagCount;
        public final double timestampSeconds;

        public VisionMeasurement(
            Pose2d pose,
            double timestamp,
            double tagDistance,
            int tagCount, 
            double timestampSeconds
        ) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.tagDistance = tagDistance;
            this.tagCount = tagCount;
            this.timestampSeconds = timestampSeconds;
        }
    }

    @Override
    public void periodic() {

        /// Debug robot loss of pose -----------------------------------------
        // double[] botpose = this.telemetryTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        // // botpose format:
        // // [x, y, z, roll, pitch, yaw] in meters + degrees

        // double llYawDeg = botpose[5];

        // Logger.recordOutput(
        //     "Localization/LimelightYawDeg",
        //     llYawDeg
        // );
        // ---------------------------------------------------------------------
    }
}
