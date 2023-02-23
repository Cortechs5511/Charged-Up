package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StartAutoAlign;
//import frc.robot.commands.drive.Limelight.GoToTag;
import frc.robot.commands.drive.Limelight.LimelightDisplay;
import frc.robot.subsystems.*;

public class RobotContainer {
    private SendableChooser<Command> chooser = new SendableChooser<>();

    private final Drive drive = new Drive();
    private final Limelight limelight = new Limelight();
    private final OI oi = OI.getInstance();

    public RobotContainer() {
        drive.setDefaultCommand(new SetSpeed(drive));
        //limelight.setDefaultCommand(new LimelightDisplay(limelight));
        configureButtonBindings();

        chooser.addOption("Test auto", trajectoryFollower("pathplanner/generatedJSON/Score 1, Pick 1, Balance.wpilib.json",drive,true));
        chooser.addOption("idklol", new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));
        Shuffleboard.getTab("Autonomous Selection").add(chooser);
    }

    private void configureButtonBindings() {
        new JoystickButton(oi.leftStick, Constants.OIConstants.FLIP_BUTTON).onTrue(new Flip(drive));
        //new JoystickButton(oi.rightStick, Constants.OIConstants.HALF_SPEED_BUTTON)
                //.onTrue(drive.setMaxPower(0.5)).onFalse(drive.setMaxPower(1.0));

        new JoystickButton(oi.leftStick, 1)
        .toggleOnTrue(new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));

        new JoystickButton(oi.rightStick, 1)
        .toggleOnTrue(getCommand(drive, limelight, 0.0));

    }

    public Command trajectoryFollower(String filename, Drive drive, boolean reset) {
        Trajectory trajectory;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("hi");
        } catch (IOException exception) {
            DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
            System.out.println("Unable to read from file" + filename);
            return new InstantCommand();
        }
        
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drive::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv,
                DriveConstants.Ka),
        DriveConstants.DRIVE_KINEMATICS, drive::getWheelSpeeds,
        new PIDController(DriveConstants.Kp, 0, 0),
        new PIDController(DriveConstants.Kp, 0, 0),
        drive::setVolts, drive);

        if (reset) {
            SmartDashboard.putString("Auto status", "working");
            return new SequentialCommandGroup(new InstantCommand(() -> drive.reset(trajectory.getInitialPose())), ramseteCommand);
        } else{
            SmartDashboard.putString("Auto status", "working");
            return ramseteCommand;
        }
    
        
    }
    public void diagnostics() {
        SmartDashboard.putNumber("leftDistance", drive.getLeftPosition());
        SmartDashboard.putNumber("rightDistance", drive.getRightPosition());
        SmartDashboard.putNumber("leftVelocity", drive.getLeftVelocity());
        SmartDashboard.putNumber("rightVelocity", drive.getRightVelocity());
      }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public Drive getDrive() {
        return drive;
    }



    public Command getCommand(Drive drive, Limelight limelight, Double sideOffset) {
        var startingPose = new Pose2d(0,0, new Rotation2d());
        

        if (limelight.hasTargets() == false) {
            return new InstantCommand();
        } else {
            double yaw = limelight.getPitch();
            double xLL = limelight.getZ();
            double yLL = -limelight.getX();
            
            Rotation2d robotFinalToRobotInitial = new Rotation2d(Units.degreesToRadians(yaw)-Math.PI);

            Translation2d originFinalToTag = new Translation2d(LimelightConstants.ORIGIN_TO_TAG_FINAL, Units.inchesToMeters(sideOffset));

            Translation2d limelightToTag = new Translation2d(xLL, yLL);

            Translation2d originToLimelight = new Translation2d(
            LimelightConstants.ORIGIN_TO_LIMELIGHT_X, 
            LimelightConstants.ORIGIN_TO_LIMELIGHT_Y);

            Translation2d originToTag = limelightToTag.plus(originToLimelight);

            Translation2d finalTranslation = originToTag.minus(originFinalToTag.rotateBy(robotFinalToRobotInitial));

            SmartDashboard.putNumber("Limelight/rotate", robotFinalToRobotInitial.getDegrees());


            Pose2d endingPose = new Pose2d(finalTranslation.getX(), finalTranslation.getY(), robotFinalToRobotInitial);

            var interiorWaypoints = new ArrayList<Translation2d>();
            interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
            interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));

            TrajectoryConfig config = new TrajectoryConfig(0.5, 0.25);
            config.setReversed(false);
            
            var trajectory = TrajectoryGenerator.generateTrajectory(
                startingPose,
                interiorWaypoints,
                endingPose,
                config);

            drive.reset(trajectory.getInitialPose());
            SmartDashboard.putNumber("Limelight/InitialRotate", drive.getPose().getRotation().getDegrees());

            SmartDashboard.putString("Limelight/Align", "3");
            
            RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drive::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka),
            DriveConstants.DRIVE_KINEMATICS, 
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.Kp, 0, 0),
            new PIDController(DriveConstants.Kp, 0, 0),
            drive::setVolts, drive);

            return ramseteCommand;

        }
    }
    
}
