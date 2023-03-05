package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class scoreHighCone extends CommandBase {
    private final Arm arm;
    private double angle;

    public scoreHighCone(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if(Math.abs(arm.getArmPosition()) < angle) {
            arm.setPower(-1.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPower(0);
    }

    @Override 
    public boolean isFinished() {
         return Math.abs(arm.getArmPosition()) >= angle;
        
    }
}
