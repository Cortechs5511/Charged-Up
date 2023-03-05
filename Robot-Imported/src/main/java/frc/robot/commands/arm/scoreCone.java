package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;


public class scoreCone extends CommandBase {
private final Arm arm;
private boolean reachedPosition = false; 
private final double angle;
   public scoreCone(Arm arm, double angle) {
    this.arm = arm;
    this.angle = angle;
    addRequirements(arm);
}

@Override
public void execute() {
    if(Math.abs(arm.getArmPosition()) < angle && !reachedPosition) {
        arm.setPower(-1.5);
    }else{
        reachedPosition = true;
        arm.setPower(-2.66*Math.cos(arm.getRadians()));
    }
}

@Override
public void end(boolean interrupted) {
    arm.setPower(0);
}
}
