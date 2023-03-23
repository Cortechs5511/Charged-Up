package frc.robot.commands.claw;

import frc.robot.OI;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class stallClaw extends CommandBase {
    private final Claw claw;
    private final OI oi;
    
    public stallClaw(Claw claw, OI oi) {
        this.claw = claw;
        this.oi = oi;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (oi.controller.getRightBumper()) {
        claw.setClawPower(-0.05);
        } else {
        claw.setClawPower(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        claw.setClawPower(0);
    }
}
