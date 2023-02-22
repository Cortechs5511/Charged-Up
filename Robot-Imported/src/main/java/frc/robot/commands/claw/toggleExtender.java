package main.java.frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class toggleGripper extends CommandBase {
    private final Claw claw;

    public toggleExtender(Claw subsystem) {
        claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        extenderSolenoid.toggle();
    }
}
