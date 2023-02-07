package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StartAutoAlign;

import frc.robot.subsystems.*;

public class RobotContainer {
    private SendableChooser<Command> chooser = new SendableChooser<>();

    private final Drive drive = new Drive();
    private final OI oi = OI.getInstance();

    public RobotContainer() {
        drive.setDefaultCommand(new SetSpeed(drive));
        configureButtonBindings();

        chooser.addOption("Test auto", trajectoryFollower("output/Score 1, Pick 1, Balance.wpilib.json",drive,true));
        chooser.addOption("idklol", new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));
        Shuffleboard.getTab("Autonomous Selection").add(chooser);
    }

    private void configureButtonBindings() {
        new JoystickButton(oi.leftStick, Constants.OIConstants.FLIP_BUTTON).onTrue(new Flip(drive));
        //new JoystickButton(oi.rightStick, Constants.OIConstants.HALF_SPEED_BUTTON)
                //.onTrue(drive.setMaxPower(0.5)).onFalse(drive.setMaxPower(1.0));

        new JoystickButton(oi.leftStick, 1)
        .toggleOnTrue(new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));
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
        new RamseteController(2, 0.7),
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

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
