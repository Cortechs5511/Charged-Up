package frc.robot.commands.arm;

import frc.robot.subsystems.Extender;
import frc.robot.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class extendArm extends CommandBase {
    private final Extender extender;
    private final OI oi = OI.getInstance();


    public extendArm(Extender subsystem) {
        extender = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (extender.getStringPotPosition() <= 6.24 && oi.controller.getRightY() < 0) {
            extender.setExtendPower(0);
        } else if (extender.getStringPotPosition() >= 80 && oi.controller.getRightY() > 0) {
            extender.setExtendPower(0);
        } else {
            extender.setExtendPower(oi.controller.getRightY());
        }
}
}
