package frc.robot.commands.drive.Limelight;

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

public class GoToTag extends CommandBase {   
    // Drive drive;
    // Limelight limelight;
    // double sideOffset;
    // RamseteController ramseteController = new RamseteController();

    // public GoToTag(Drive drive, Limelight limelight, double sideOffset) {
    //     this.limelight = limelight;
    //     this.drive = drive;
    //     this.sideOffset = sideOffset;
    //     addRequirements(drive, limelight);
    // }

    public RamseteCommand alignCommand(Drive drive, Limelight limelight, double sideOffset) {
        limelight.setSideOffset(sideOffset);
            
        Trajectory trajectory = limelight.getTrajectory();
        
        limelight.setFlag();


        drive.reset(trajectory.getInitialPose());
    
        // Push the trajectory to Field2d.
        drive.getField2d().getObject("traj").setTrajectory(trajectory);

            
            return new RamseteCommand(trajectory, drive::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka),
            DriveConstants.DRIVE_KINEMATICS, 
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.Kp, 0, 0),
            new PIDController(DriveConstants.Kp, 0, 0),
            drive::setVolts, drive);



        
    }
}
