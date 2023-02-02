package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.Flip;
import frc.robot.commands.drive.SetSpeed;
import frc.robot.commands.drive.StartAutoAlign;
import frc.robot.subsystems.*;

public class RobotContainer {
    private SendableChooser<AutoRoutine> chooser = new SendableChooser<>();

    private final Drive drive = new Drive();
    private final OI oi = OI.getInstance();

    enum AutoRoutine {
        WaitCommand, TwoBallAuto, TwoBallAutoAlt, OneBallAuto
    }

    public RobotContainer() {
        drive.setDefaultCommand(new SetSpeed(drive));
        configureButtonBindings();

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
        AutoRoutine choice = chooser.getSelected();
        return null;
    }
}
