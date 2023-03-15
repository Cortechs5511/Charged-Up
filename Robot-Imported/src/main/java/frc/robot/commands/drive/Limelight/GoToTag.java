package frc.robot.commands.drive.Limelight;

import java.sql.Driver;
import java.util.ArrayList;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.LimelightHelpers;

public class GoToTag extends CommandBase {   
    Drive drive;
    Limelight limelight;
    double sideOffset;
    RamseteController ramseteController = new RamseteController();
    // LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    RamseteCommand ramseteCommand;
    private Pose2d startingPose = new Pose2d(0,0, new Rotation2d(0));


    public GoToTag(Drive drive, Limelight limelight, double sideOffset) {
        this.limelight = limelight;
        this.drive = drive;
        this.sideOffset = sideOffset;
        addRequirements(drive, limelight);
    }

    @Override
    public void initialize(){

}

    @Override
    public void execute(){

        if (limelight.hasTargets()) {

            double rotate = limelight.getPitch();
            double xLL = limelight.getZ();
            double yLL = -limelight.getX();

            //1
            System.out.print("1");

            Rotation2d robotFinalToRobotInitial = new Rotation2d(Units.degreesToRadians(rotate));

            Translation2d originFinalToTag = new Translation2d(LimelightConstants.ORIGIN_TO_TAG_FINAL, Units.inchesToMeters(sideOffset));

            Translation2d limelightToTag = new Translation2d(xLL, yLL);

            Translation2d originToLimelight = new Translation2d(
            LimelightConstants.ORIGIN_TO_LIMELIGHT_X, 
            LimelightConstants.ORIGIN_TO_LIMELIGHT_Y);

            Translation2d originToTag = limelightToTag.plus(originToLimelight);

            Translation2d finalTranslation = originToTag.minus(originFinalToTag.rotateBy(robotFinalToRobotInitial));

            SmartDashboard.putNumber("Limelight/rotate", -limelight.getPitch());
            SmartDashboard.putNumber("Limelight/translateX", finalTranslation.getX());
            SmartDashboard.putNumber("Limelight/translateY", finalTranslation.getX());

            //2
            System.out.print("2");

            Pose2d endingPose = new Pose2d(finalTranslation.getX(), finalTranslation.getY(), new Rotation2d());

            SmartDashboard.putString("Limelight/Endingpose", endingPose.toString());
            SmartDashboard.putString("Limelight/Startpose", startingPose.toString());

            //3
            System.out.print("3");

            var interiorWaypoints = new ArrayList<Translation2d>();
            interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
            interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));

            //4
            System.out.print("4");

            TrajectoryConfig config = new TrajectoryConfig(0.5, 0.25);
            config.setReversed(false);
            
            Trajectory traj = TrajectoryGenerator.generateTrajectory(
                startingPose,
                interiorWaypoints,
                endingPose,
                config);

            //5
            System.out.print("5");



            ramseteCommand = new RamseteCommand(traj, drive::getPose,
            ramseteController,
            new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka),
            DriveConstants.DRIVE_KINEMATICS, 
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.Kp, 0, 0),
            new PIDController(DriveConstants.Kp, 0, 0),
            drive::setVolts, drive);

            //6
            System.out.print("6");

            ramseteCommand.execute();
    }
    }


    @Override
    public void end(boolean interrupted) {
        drive.setPower(0,0);

    }

    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }
}
