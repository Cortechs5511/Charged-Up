package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class scoreHighCone extends CommandBase {
    private final Arm arm;

    public scoreHighCone(Arm subsystem) {
        arm = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(arm.getArmPosition()) < ArmConstants.HIGH_CONE_ROTATIONS) {
            arm.setPower(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPower(0);
    }

    @Override 
    public boolean isFinished() {
        if (Math.abs(arm.getArmPosition()) >= ArmConstants.HIGH_CONE_ROTATIONS) {
            return true;
        } else{
        return false;
        }
    }
}
