package frc.robot.commands.drive.Limelight;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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

public class GoToTag extends CommandBase {   
    Drive drive;
    Limelight limelight;
    double sideOffset;
    RamseteController ramseteController = new RamseteController();
    //LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    RamseteCommand ramseteCommand;
    Trajectory traj;
    private Pose2d startingPose = new Pose2d(0,0, new Rotation2d(0));


    public GoToTag(Drive drive, Limelight limelight, Trajectory traj) {
        this.limelight = limelight;
        this.drive = drive;
        this.traj = traj;
        addRequirements(drive, limelight);
    }

    @Override
    public void initialize(){

}

    @Override
    public void execute(){

            ramseteCommand = new RamseteCommand(traj, drive::getPose,
            ramseteController,
            new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka),
            DriveConstants.DRIVE_KINEMATICS, 
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.Kp, 0, 0),
            new PIDController(DriveConstants.Kp, 0, 0),
            drive::setVolts, drive);


            ramseteCommand.execute();
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
