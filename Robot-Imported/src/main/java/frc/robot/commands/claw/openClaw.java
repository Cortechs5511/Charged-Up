package frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class openClaw extends InstantCommand {
    private final Claw claw;

    public openClaw(Claw subsystem) {
        claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        claw.setGripper(true);
        claw.setExtender(false);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
