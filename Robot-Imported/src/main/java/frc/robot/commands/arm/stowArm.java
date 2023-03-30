package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class stowArm extends CommandBase {
    private final Arm arm;
    private final Extender extender;


    public stowArm(Arm arm, Extender extender) {
        this.arm = arm;
        this.extender = extender;
        addRequirements(arm, extender);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {

        if (Math.abs(arm.getArmPosition()) < ArmConstants.STOW_ROTATIONS - ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(0.25);
        } else if(Math.abs(arm.getArmPosition()) > ArmConstants.STOW_ROTATIONS + ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(-0.8);
        }
        extender.goToPosition(0.8, ArmConstants.ZERO_EXTENSION);
        }

    
    @Override
    public void end(boolean interrupted) {
        arm.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return //ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE <= extender.getStringPotPosition() 
        //&& extender.getStringPotPosition() <= ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE
        //&& 
        ArmConstants.STOW_ROTATIONS - ArmConstants.ARM_SCORE_TOLERANCE <= arm.getArmPosition() 
        && arm.getArmPosition() <= ArmConstants.STOW_ROTATIONS + ArmConstants.ARM_SCORE_TOLERANCE;
    }
}

