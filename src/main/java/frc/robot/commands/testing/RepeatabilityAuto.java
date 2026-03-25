// package frc.robot.commands.testing;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Landmarks;
// import frc.robot.subsystems.Swerve;
// import frc.util.RepeatabilityLogger;

// public class RepeatabilityAuto extends SequentialCommandGroup {

//     public RepeatabilityAuto(Swerve drivetrain,
//                              Command AtoB,
//                              Command BtoA) {

//         Translation2d goalPose = Landmarks.hubPosition();

//         RepeatabilityLogger logger =
//                 new RepeatabilityLogger("repeatability_test.csv");

//         for (int i = 0; i < 10; i++) {

//             addCommands(
//                     AtoB,
//                     new WaitUntilCommand(() -> drivetrain.isNearZeroVelocity()),
//                     new LogCommand(drivetrain, logger, goalPose),
                    
//                     BtoA,
//                     new WaitUntilCommand(() -> drivetrain.isNearZeroVelocity()),
//                     new LogCommand(drivetrain, logger, goalPose)
//             );
//         }

//         addCommands(new InstantCommand(logger::close));
//     }
// }