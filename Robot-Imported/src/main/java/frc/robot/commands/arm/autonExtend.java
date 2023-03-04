package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Extender;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autonExtend extends CommandBase {
    private final Extender extender;

    public autonExtend(Extender subsystem) {
        extender = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(extender.getExtenderPostion()) > ArmConstants.AUTON_EXTEND_POS) {
            extender.setExtendPower(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        extender.setExtendPower(0);
    }

    @Override 
    public boolean isFinished() {
        if (Math.abs(extender.getExtenderPostion()) <= ArmConstants.AUTON_EXTEND_POS) {
            return true;
        } else{
        return false;
        }
    }

}
