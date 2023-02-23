// package frc.robot.commands.drive.Limelight;

// import java.util.ArrayList;

// import javax.xml.crypto.dsig.Transform;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.lib.util.FieldConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.LimelightConstants;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Limelight;

// public class GoToTag extends SequentialCommandGroup {
    
//     Drive drive;
//     Limelight limelight;
//     double sideOffset;
//     RamseteController ramseteController = new RamseteController();
//     Pose2d pose2d = new Pose2d();

//     public GoToTag(Drive drive, Limelight limelight, double sideOffset) {
//         this.limelight = limelight;
//         this.drive = drive;
//         this.sideOffset = sideOffset;
//         addRequirements(drive, limelight);
//     }

//     public Command getCommand() {
//         var startingPose = new Pose2d(0,0, new Rotation2d());
        

//         if (limelight.hasTargets() == false) {
//             return new InstantCommand();
//         } else {
//             double yaw = limelight.getPitch();
//             double xLL = limelight.getZ();
//             double yLL = limelight.getX();
            
//             Rotation2d robotFinalToRobotInitial = new Rotation2d(yaw - Math.PI);

//             Translation2d originFinalToTag = new Translation2d(LimelightConstants.ORIGIN_TO_TAG_FINAL, Units.inchesToMeters(sideOffset));

//             Translation2d limelightToTag = new Translation2d(xLL, yLL);

//             Translation2d originToLimelight = new Translation2d(
//             LimelightConstants.ORIGIN_TO_LIMELIGHT_X, 
//             LimelightConstants.ORIGIN_TO_LIMELIGHT_Y);

//             Translation2d originToTag = limelightToTag.plus(originToLimelight);

//             Translation2d finalTranslation = originToTag.minus(originFinalToTag.rotateBy(robotFinalToRobotInitial));

//             SmartDashboard.putString("Limelight/Align", "2");


//             Pose2d endingPose = new Pose2d(finalTranslation.getX(), finalTranslation.getY(), robotFinalToRobotInitial);

//             var interiorWaypoints = new ArrayList<Translation2d>();
//             interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
//             interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));

//             TrajectoryConfig config = new TrajectoryConfig(1, 1);
//             config.setReversed(false);
            
//             var trajectory = TrajectoryGenerator.generateTrajectory(
//                 startingPose,
//                 interiorWaypoints,
//                 endingPose,
//                 config);

//             drive.reset(trajectory.getInitialPose());

//             SmartDashboard.putString("Limelight/Align", "3");
            
//             return new RamseteCommand(trajectory, drive::getPose,
//             new RamseteController(),
//             new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka),
//             DriveConstants.DRIVE_KINEMATICS, 
//             drive::getWheelSpeeds,
//             new PIDController(DriveConstants.Kp, 0, 0),
//             new PIDController(DriveConstants.Kp, 0, 0),
//             drive::setVolts, drive);



//         }
//     }
// }
