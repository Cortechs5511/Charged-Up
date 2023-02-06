package frc.robot;

import java.util.*;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StartAutoAlign;
import frc.robot.commands.drive.TrajectoryFollower;

import frc.robot.subsystems.*;

public class RobotContainer {
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<>();

    private final Drive drive = new Drive();
    private final OI oi = OI.getInstance();

    enum AutoRoutine {
        WaitCommand, Score1Pick1Balance, StartAlign
    }

    public RobotContainer() {
        drive.setDefaultCommand(new SetSpeed(drive));
        configureButtonBindings();

        chooser.addOption("Wait Command", AutoRoutine.WaitCommand);
        chooser.addOption("Test auto", AutoRoutine.Score1Pick1Balance);
        chooser.addOption("idklol", AutoRoutine.StartAlign);
        Shuffleboard.getTab("Autonomous Selection").add(chooser);
    }

    private void configureButtonBindings() {
        new JoystickButton(oi.leftStick, Constants.OIConstants.FLIP_BUTTON).onTrue(new Flip(drive));
        //new JoystickButton(oi.rightStick, Constants.OIConstants.HALF_SPEED_BUTTON)
                //.onTrue(drive.setMaxPower(0.5)).onFalse(drive.setMaxPower(1.0));

        new JoystickButton(oi.leftStick, 1)
        .toggleOnTrue(new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));
    }

    public Command getAutonomousCommand() {
        var pathGroup1 = PathPlanner.loadPathGroup("Score 1, Pick 1, Balance", new PathConstraints(0.2, 0.2));

        HashMap <String, Command> eventMap = new HashMap<>();

        eventMap.put("StartBalance", new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive))));
        
        AutoRoutine choice = chooser.getSelected();
        Command selected;

        switch (choice) {
            case Score1Pick1Balance:
                selected = TrajectoryFollower.getPath(pathGroup1.get(0), drive, true);
                break;
            case StartAlign:
                selected = new SequentialCommandGroup(new StartAutoAlign(drive).andThen(new AutoAlign(drive)));
                break;
            default:
                selected = new WaitCommand(1.0);
                break;
            
    } return selected;
}
}
