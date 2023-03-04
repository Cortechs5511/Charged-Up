package frc.robot.commands.arm;

import frc.robot.subsystems.Extender;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class armExtend extends CommandBase {
    private final Extender extender;
    private final Timer timer= new Timer();
    private double power;


    public armExtend(Extender extender, double power) {
        this.extender = extender;
        this.power = power;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        if (timer.hasElapsed(0.25)) {
        extender.setExtendPower(power);
        }
        else{
            extender.setExtendPower(0);
        }
        extender.zero();
}

@Override
public boolean isFinished() {
    return timer.hasElapsed(0.25);
}
}
