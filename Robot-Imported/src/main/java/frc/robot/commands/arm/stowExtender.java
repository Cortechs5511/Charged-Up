package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class stowExtender extends CommandBase {
    private final Extender extender;

    public stowExtender( Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {


        if (extender.getExtenderPosition() < ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE) {
            extender.setExtendPower(0.8);
        } else if(extender.getExtenderPosition() > ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE) {
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