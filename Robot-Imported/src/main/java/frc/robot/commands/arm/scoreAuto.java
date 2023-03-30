package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.Constants.ArmConstants;


public class scoreAuto extends CommandBase {
    private final Arm arm;
    private final Extender extender;
    private final double angle;
    private final double passivePower;
    private final double extension;
    
    public scoreAuto(Extender extender, Arm arm, double angle, double passivePower, double extension) {
        this.extender = extender;
        this.arm = arm;
        this.angle = angle;
        this.passivePower = passivePower;
        this.extension = extension;
        addRequirements(arm);
        addRequirements(extender);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(Math.abs(arm.getArmPosition()) < angle + ArmConstants.ARM_SCORE_TOLERANCE && Math.abs(arm.getArmPosition()) > angle - ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(passivePower);
        } else if (Math.abs(arm.getArmPosition()) < angle - ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(0.8);
        } else if(Math.abs(arm.getArmPosition()) > angle + ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(-0.25);
        }
        extender.goToPosition(0.8, extension);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPower(0);
        extender.setExtendPower(0);
    }

    @Override 
    public boolean isFinished() {
        return //extension - ArmConstants.EXTENDER_SCORE_TOLERANCE <= extender.getStringPotPosition()
        //&& extender.getStringPotPosition() <= extension + ArmConstants.EXTENDER_SCORE_TOLERANCE
        //&&  
        angle - ArmConstants.ARM_SCORE_TOLERANCE <= arm.getArmPosition() 
        && arm.getArmPosition() <= angle + ArmConstants.ARM_SCORE_TOLERANCE;
    }
}
