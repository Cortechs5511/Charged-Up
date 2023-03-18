package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class runClawTime extends CommandBase {
    private final Claw claw;
    private final Timer timer = new Timer();
    private final double time;
    
    public runClawTime(Claw claw, double time) {
        this.claw = claw;
        this.time = time;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        timer.reset();
        timer.start();
        if (!timer.hasElapsed(time)){
            claw.setClawPower(1);
        } else {
            claw.setClawPower(0);
        }

        
}

    @Override
    public void end(boolean interrupted) {
        claw.setClawPower(0);
    }

    @Override 
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

}
