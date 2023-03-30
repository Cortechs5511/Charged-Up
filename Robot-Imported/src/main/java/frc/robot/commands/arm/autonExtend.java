package frc.robot.commands.arm;

import frc.robot.subsystems.Extender;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autonExtend extends CommandBase {
    private final Extender extender;
    private final double extension;

    public autonExtend(Extender subsystem, double extension) {
        extender = subsystem;
        this.extension = extension;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (extender.getStringPotPosition() > extension) {
            extender.setExtendPower(0.8);
        } else if (extender.getStringPotPosition() < extension) {
            extender.setExtendPower(-0.8);
        } else {
            extender.setExtendPower(0);
        }
}

}
