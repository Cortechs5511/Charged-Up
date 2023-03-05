package frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class closeClaw extends InstantCommand {
    private final Claw claw;

    public closeClaw(Claw subsystem) {
        claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        claw.setExtender(true);
        claw.setGripper(false);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
