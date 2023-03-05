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
import frc.robot.commands.arm.armExtend;
import frc.robot.commands.arm.autonExtend;
import frc.robot.commands.arm.autonRetract;
import frc.robot.commands.arm.extendArm;
import frc.robot.commands.arm.scoreCone;
import frc.robot.commands.arm.setArmPower;
import frc.robot.commands.arm.stowArm;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.DriveForTime;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetMaxPower;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StartAutoAlign;
import frc.robot.subsystems.*;
import frc.robot.commands.claw.closeClaw;
import frc.robot.commands.claw.extenderForward;
import frc.robot.commands.claw.extenderReverse;
import frc.robot.commands.claw.gripperForward;
import frc.robot.commands.claw.gripperReverse;
import frc.robot.commands.claw.openClaw;
import frc.robot.commands.arm.scoreHighCone;

public class RobotContainer {
    private SendableChooser<Command> chooser = new SendableChooser<>();

    private final Drive drive = new Drive();
    private final Claw claw = new Claw();
    private final Arm arm = new Arm();
    private final Extender extender = new Extender();
    private final Limelight limelight = new Limelight();
    private final OI oi = OI.getInstance();

    public RobotContainer() {
        drive.setDefaultCommand(new SetSpeed(drive));
        arm.setDefaultCommand(new setArmPower(arm));
        extender.setDefaultCommand(new extendArm(extender));
      //limelight.setDefaultCommand(new LimelightDisplay(limelight));
        configureButtonBindings();

        chooser.addOption
        ("Score+Balance", 
        (new SequentialCommandGroup(new armExtend(extender, 0.8)).andThen(new closeClaw(claw)).andThen(new scoreHighCone(arm, ArmConstants.HIGH_CONE_ROTATIONS)).andThen(new autonExtend(extender)).andThen(new openClaw(claw)).andThen(new stowArm(arm))
        .andThen(trajectoryFollower("pathplanner/generatedJSON/Leave+Balance.wpilib.json",drive,true)
        .andThen(new StartAutoAlign(drive).andThen(new AutoAlign(drive))))));

        chooser.addOption ("Leave+Balance (no score)", trajectoryFollower("pathplanner/generatedJSON/Leave+Balance.wpilib.json",drive,true)
        .andThen(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));

        chooser.addOption("Only auto balance", new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));

        chooser.addOption("Auto mobility (side auto)", trajectoryFollower("pathplanner/generatedJSON/Auto mobility.wpilib.json", drive, true));
        //chooser.addOption("Score+Auto mobility", new SequentialCommandGroup(new armExtend(extender, 0.3)).andThen(new scoreHighCone(arm, ArmConstants.INITIAL_ROTATE)).andThen(new closeClaw(claw)).andThen(new scoreHighCone(arm, ArmConstants.EXTENDABLE_ROTATIONS).andThen(new autonExtend(extender)).andThen(new scoreHighCone(arm, ArmConstants.HIGH_CONE_ROTATIONS)).andThen(new DriveForTime(drive, 0.5)).andThen(new openClaw(claw)).andThen(new autonRetract(extender)).andThen
        //(new stowArm(arm)).andThen(trajectoryFollower("pathplanner/generatedJSON/Auto mobility.wpilib.json", drive, true))));


        chooser.addOption("test", (new SequentialCommandGroup(new armExtend(extender, 0.8)).andThen(new closeClaw(claw)).andThen(new scoreHighCone(arm, ArmConstants.HIGH_CONE_ROTATIONS)).andThen(new autonExtend(extender)).andThen(new openClaw(claw)).andThen(new stowArm(arm))));
        Shuffleboard.getTab("Autonomous Selection").add(chooser);

    }
 
    private void configureButtonBindings() {
        new JoystickButton(oi.leftStick, Constants.OIConstants.FLIP_BUTTON).onTrue(new Flip(drive));
        new JoystickButton(oi.rightStick, Constants.OIConstants.HALF_SPEED_BUTTON)
                .onTrue(new SetMaxPower(drive, 0.5)).onFalse(new SetMaxPower(drive, 1.0));

        new JoystickButton(oi.leftStick, 1)
        //.toggleOnTrue(new AutoAlign(drive));
        .toggleOnTrue(new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));

        new JoystickButton(oi.rightStick, 1)
        .toggleOnTrue(new SequentialCommandGroup(new armExtend(extender, 0.8)).andThen(new closeClaw(claw)).andThen(new scoreHighCone(arm, ArmConstants.HIGH_CONE_ROTATIONS)).andThen(new openClaw(claw)).andThen(new stowArm(arm)));
        //.toggleOnTrue(alignCommand(drive, limelight, 0.0));
        //.toggleOnTrue(new SequentialCommandGroup(new TurnByAngle(drive, -limelight.getPitch()).andThen(getCommand(drive, limelight, 0.0))));

        // Claw commands, open claw, grab cube, grab cone
        // new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).x().onTrue(new gripperForward(claw));
        // new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).b().onTrue(new extenderForward(claw));
        // new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).b().onTrue(new gripperReverse(claw));
        // new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).x().onTrue(new extenderReverse(claw));

        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).rightTrigger().onTrue(new openClaw(claw));
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).leftTrigger().onTrue(new closeClaw(claw));
        
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).y().toggleOnTrue(new scoreCone(arm, ArmConstants.HIGH_CONE_ROTATIONS));
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).a().toggleOnTrue(new scoreCone(arm, ArmConstants.SUBSTATION_ROTATIONS));
        new CommandXboxController(OIConstants.XBOX_CONTROLLER_PORT).x().whileTrue(Commands.run(() -> arm.setPower(-2*Math.cos(arm.getRadians())), arm));



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



    public Command alignCommand(Drive drive, Limelight limelight, Double sideOffset) {
        var startingPose = new Pose2d(0,0, new Rotation2d());
        

        if (limelight.hasTargets() == false) {
            return new InstantCommand();
        } else {
            double rotate = limelight.getPitch();
            double xLL = -limelight.getZ();
            double yLL = -limelight.getX();
            
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




            Pose2d endingPose = new Pose2d(finalTranslation.getX(), finalTranslation.getY(), new Rotation2d());

            SmartDashboard.putString("Limelight/Endingpose", endingPose.toString());
            SmartDashboard.putString("Limelight/Startpose", startingPose.toString());

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
        
            // Push the trajectory to Field2d.
            drive.getField2d().getObject("traj").setTrajectory(trajectory);

            RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drive::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka),
            DriveConstants.DRIVE_KINEMATICS, 
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.Kp, 0, 0),
            new PIDController(DriveConstants.Kp, 0, 0),
            drive::setVolts, drive);
//hi
            return ramseteCommand;

        }
    }
    }
    

