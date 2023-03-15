package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class retractArm extends CommandBase {
    private final Extender extender;


    public retractArm(Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (Math.abs(extender.getExtenderPosition()) < ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE) {
            extender.setExtendPower(0.8);
        } else if(Math.abs(extender.getExtenderPosition()) > ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE) {
            extender.setExtendPower(-0.25);
        }
    }

    @Override
    public void end(boolean interrupted) {
        extender.setExtendPower(0);
    }


    @Override 
    public boolean isFinished() {
            return ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE <= extender.getExtenderPosition() 
            && extender.getExtenderPosition() <= ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE;
        
    }
}
