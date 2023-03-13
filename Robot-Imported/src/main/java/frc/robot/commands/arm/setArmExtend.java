package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Extender;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class setArmExtend extends CommandBase {
    private final Extender extender;
    private double extension;

    public setArmExtend(Extender subsystem, double extension) {
        extender = subsystem;
        this.extension = extension;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(extender.getExtenderPostion()) > extension) {
            extender.setExtendPower(-1);
        } else {
            extender.setExtendPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        extender.setExtendPower(0);
    }

    @Override 
    public boolean isFinished() {
        return (extender.getExtenderPostion() >= extension-0.05);
    }

}
