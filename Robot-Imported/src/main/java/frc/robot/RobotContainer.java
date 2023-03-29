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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.extendArm;
import frc.robot.commands.arm.scoreAuto;
import frc.robot.commands.arm.scoreCone;
import frc.robot.commands.arm.setArmPower;
import frc.robot.commands.arm.stowArm;
import frc.robot.commands.arm.stowExtender;
import frc.robot.commands.arm.retractArm;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.DriveForTime;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetMaxPower;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StartAutoAlign;
import frc.robot.commands.drive.goToSubstation;
//import frc.robot.commands.drive.TurnByAngle;
import frc.robot.subsystems.*;
import frc.robot.commands.claw.manipulateClaw;
import frc.robot.commands.claw.runClawTime;
import frc.robot.commands.claw.stallClaw;

public class RobotContainer {
    private SendableChooser<Command> chooser = new SendableChooser<>();

    private final Drive drive = new Drive();
    private final Arm arm = new Arm();
    private final Extender extender = new Extender();
    private final Limelight limelight = new Limelight();
    private final Claw claw = new Claw();
    private final OI oi = OI.getInstance();

    public RobotContainer() {
        drive.setDefaultCommand(new SetSpeed(drive));
        arm.setDefaultCommand(new setArmPower(arm));
        extender.setDefaultCommand(new extendArm(extender));
        claw.setDefaultCommand(new manipulateClaw(claw, oi));
      //limelight.setDefaultCommand(new LimelightDisplay(limelight));
        configureButtonBindings();

        // chooser.addOption
        // ("Score+Balance", 
        // (new SequentialCommandGroup(new armExtend(extender, 0.8)).andThen(new closeClaw(claw)).andThen(new scoreHighCone(arm, ArmConstants.HIGH_CONE_ROTATIONS)).andThen(new autonExtend(extender)).andThen(new openClaw(claw)).andThen(new stowArm(arm))
        // .andThen(trajectoryFollower("pathplanner/generatedJSON/Leave+Balance.wpilib.json",drive,true)
        // .andThen(new StartAutoAlign(drive).andThen(new AutoAlign(drive))))));

        chooser.addOption ("Leave+Balance (no score)", new SequentialCommandGroup(new DriveForTime(drive, 3, -0.6).withTimeout(2.5)).andThen(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));

        chooser.addOption("Score+ balance", new SequentialCommandGroup(new retractArm(extender).withTimeout(1)).andThen(new scoreAuto(extender, arm, ArmConstants.HIGH_CONE_ROTATIONS, ArmConstants.HIGH_POWER, ArmConstants.MID_EXTENSION).withTimeout(4))
        .andThen(new runClawTime(claw, 0.2).withTimeout(0.2))
        .andThen(new stowArm(arm,extender).withTimeout(4)).andThen(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));
        
        chooser.addOption("Nothing", new InstantCommand());
        chooser.addOption("Balance", new SequentialCommandGroup(new StartAutoAlign(drive)).andThen(new AutoAlign(drive)));
        chooser.addOption("Auto mobility", new DriveForTime(drive, 3, 0.2).withTimeout(3));
        //chooser.addOption("Score+Auto mobility", new SequentialCommandGroup(new armExtend(extender, 0.3)).andThen(new scoreHighCone(arm, ArmConstants.INITIAL_ROTATE)).andThen(new closeClaw(claw)).andThen(new scoreHighCone(arm, ArmConstants.EXTENDABLE_ROTATIONS).andThen(new autonExtend(extender)).andThen(new scoreHighCone(arm, ArmConstants.HIGH_CONE_ROTATIONS)).andThen(new DriveForTime(drive, 0.5)).andThen(new openClaw(claw)).andThen(new autonRetract(extender)).andThen
        //(new stowArm(arm)).andThen(trajectoryFollower("pathplanner/generatedJSON/Auto mobility.wpilib.json", drive, true))));

        chooser.addOption("Auto mobility + score", new SequentialCommandGroup(new retractArm(extender).withTimeout(1)).andThen(new scoreAuto(extender, arm, ArmConstants.HIGH_CONE_ROTATIONS, ArmConstants.HIGH_POWER, ArmConstants.MID_EXTENSION).withTimeout(4))
        .andThen(new runClawTime(claw, 1).withTimeout(1))
        .andThen(new stowArm(arm,extender).withTimeout(4)).andThen(new DriveForTime(drive, 3, 0.5).withTimeout(3)));
        Shuffleboard.getTab("Autonomous Selection").add(chooser);

    }
 
    private void configureButtonBindings() {
        new JoystickButton(oi.leftStick, Constants.OIConstants.FLIP_BUTTON).onTrue(new Flip(drive));
        new JoystickButton(oi.rightStick, Constants.OIConstants.HALF_SPEED_BUTTON)
                .onTrue(new SetMaxPower(drive, 0.5)).onFalse(new SetMaxPower(drive, 1.0));

        new JoystickButton(oi.leftStick, 1)
        .toggleOnTrue(new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));

        new JoystickButton(oi.rightStick, 1)
        //.toggleOnTrue(new goToSubstation(drive, claw));
        //.toggleOnTrue(Commands.runOnce(() -> limelight.GoToTag(0)));
       .toggleOnTrue(new SequentialCommandGroup(Commands.runOnce(() -> limelight.GoToTag(0))).andThen(tagTrajectoryCommand(limelight.getTrajectory(), drive)));

       //new JoystickButton(oi.rightStick, 1)
       //.toggleOnTrue(new TurnByAngle(drive, 10));
        //.toggleOnTrue(new GoToTag(drive, limelight, 0));
        //.toggleOnTrue(new SequentialCommandGroup(new TurnByAngle(drive, -limelight.getPitch()).andThen(getCommand(drive, limelight, 0.0))));


        
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).y().toggleOnTrue(new scoreCone(arm, extender, ArmConstants.HIGH_CONE_ROTATIONS, ArmConstants.HIGH_POWER, ArmConstants.HIGH_EXTENSION));
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).b().toggleOnTrue(new scoreCone(arm, extender, ArmConstants.MID_CONE_ROTATIONS, ArmConstants.MID_POWER, ArmConstants.MID_EXTENSION));
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).x().toggleOnTrue(new scoreCone(arm, extender, ArmConstants.SUBSTATION_ROTATIONS, ArmConstants.SUBSTATION_POWER, ArmConstants.SUBSTATION_EXTENSION));
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).a().toggleOnTrue(new scoreCone(arm, extender, ArmConstants.LOW_CONE_ROTATIONS, ArmConstants.LOW_POWER, ArmConstants.LOW_EXTENSION));

        // Claw commands

        //new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).rightBumper().toggleOnTrue(new stallClaw(claw));

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


    public Command tagTrajectoryCommand(Trajectory traj, Drive drive) {

        RamseteCommand ramseteCommand = new RamseteCommand(traj, drive::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv,
                DriveConstants.Ka),
        DriveConstants.DRIVE_KINEMATICS, drive::getWheelSpeeds,
        new PIDController(DriveConstants.Kp, 0, 0),
        new PIDController(DriveConstants.Kp, 0, 0),
        drive::setVolts, drive);

        drive.reset(new Pose2d());

        return ramseteCommand;



    }
    public void diagnostics() {
        // SmartDashboard.putNumber("leftDistance", drive.getLeftPosition());
        // SmartDashboard.putNumber("rightDistance", drive.getRightPosition());
        // SmartDashboard.putNumber("leftVelocity", drive.getLeftVelocity());
        // SmartDashboard.putNumber("rightVelocity", drive.getRightVelocity());
      }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public Drive getDrive() {
        return drive;
    }


}

