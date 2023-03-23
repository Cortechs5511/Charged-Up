package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class stowArm extends CommandBase {
    private final Arm arm;


    public stowArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {

        if (Math.abs(arm.getArmPosition()) < 0 - ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(0.25);
        } else if(Math.abs(arm.getArmPosition()) > 0 + ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(-0.8);
        }
        // if (extender.getExtenderPosition() < ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE) {
        //     extender.setExtendPower(0.8);
        // } else if(extender.getExtenderPosition() > ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE) {
        //     extender.setExtendPower(-0.25);
        // }
        // }
        }

    
    @Override
    public void end(boolean interrupted) {
        arm.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return //ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE <= extender.getExtenderPosition() 
        // && extender.getExtenderPosition() <= ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE
        // && 
        0 - ArmConstants.ARM_SCORE_TOLERANCE <= arm.getArmPosition() 
        && arm.getArmPosition() <= 0 + ArmConstants.ARM_SCORE_TOLERANCE;
    }
}

