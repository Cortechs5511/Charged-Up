package frc.robot.commands.claw;

import frc.robot.OI;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class manipulateClaw extends CommandBase {
    private final Claw claw;
    private final OI oi;
    
    public manipulateClaw(Claw claw, OI oi) {
        this.claw = claw;
        this.oi = oi;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (oi.leftTrigger() > 0) {
            claw.setClawPower(oi.leftTrigger());
        }
        else if (oi.rightTrigger()> 0) {
            claw.setClawPower(-1);
        } else  if (oi.controller.getRightBumper()) {
            claw.setClawPower(-0.05);
        
        } else if (oi.controller.getLeftBumper()) {
            claw.setClawPower(0.05);
        } else {
            claw.setClawPower(0);
        }
        
}
}