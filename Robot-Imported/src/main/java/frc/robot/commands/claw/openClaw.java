package frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class openClaw extends CommandBase {
    private final Claw claw;

    public openClaw(Claw subsystem) {
        claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        claw.setExtender(true);
        claw.setGripper(false);
    }
}
