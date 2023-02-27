package frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class gripperForward extends CommandBase {
    private final Claw claw;

    public gripperForward(Claw subsystem) {
        claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        claw.setGripper(true);
    }
}
