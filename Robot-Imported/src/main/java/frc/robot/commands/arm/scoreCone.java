package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.Constants.ArmConstants;


public class scoreCone extends CommandBase {
    private final Arm arm;
    private final Extender extender;
    private final double angle;
    private final double passivePower;
    private final PIDController armController = new PIDController(ArmConstants.ROTATE_Kp, 0, ArmConstants.ROTATE_Kd);
    private final double extension;
    
    public scoreCone(Arm arm, Extender extender, double angle, double passivePower, double extension) {
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
        armController.setSetpoint(angle);
        armController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        if(Math.abs(arm.getArmPosition()) < angle + ArmConstants.ARM_SCORE_TOLERANCE && Math.abs(arm.getArmPosition()) > angle - ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(passivePower);
        } else if (Math.abs(arm.getArmPosition()) < angle - ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(0.5);
        } else if(Math.abs(arm.getArmPosition()) > angle + ArmConstants.ARM_SCORE_TOLERANCE) {
            arm.setPower(-0.25);
        }
        //else {
      //      arm.setPower(armController.calculate(arm.getArmPosition()));

        //}


        
        // if(Math.abs(extender.getExtenderPosition()) < extension + ArmConstants.EXTENDER_SCORE_TOLERANCE && Math.abs(extender.getExtenderPosition()) > extension - ArmConstants.EXTENDER_SCORE_TOLERANCE) {
        //     extender.setExtendPower(0);
        // } else if (Math.abs(extender.getExtenderPosition()) < extension - ArmConstants.ARM_SCORE_TOLERANCE) {
        //     extender.setExtendPower(-0.8);
        // } else if(Math.abs(extender.getExtenderPosition()) > extension + ArmConstants.ARM_SCORE_TOLERANCE) {
        //     extender.setExtendPower(0.25);
        // }
        extender.goToPosition(0.3, extension);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPower(0);
        extender.setExtendPower(0);
    }
}
