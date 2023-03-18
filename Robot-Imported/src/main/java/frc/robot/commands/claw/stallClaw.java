package frc.robot.commands.claw;

import frc.robot.OI;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class stallClaw extends CommandBase {
    private final Claw claw;
    
    public stallClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setClawPower(-0.05);
    }

    @Override
    public void end(boolean interrupted){
        claw.setClawPower(0);
    }
}
