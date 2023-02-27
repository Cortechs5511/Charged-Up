package frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class gripperReverse extends CommandBase {
    private final Claw claw;

    public gripperReverse(Claw subsystem) {
        claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        claw.setGripper(false);
    }
}
