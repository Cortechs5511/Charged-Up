package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;
import frc.robot.OI;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class setArmPower extends CommandBase {
    private final Arm arm;
    private final OI oi = OI.getInstance();


    public setArmPower(Arm subsystem) {
        arm = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       //if (oi.getArmPower() != 0){
        arm.setPower(-oi.getArmPower());
     //   } else {
      //arm.setPower(-2*Math.cos(Math.abs(arm.getRadians())));
     //  }
        SmartDashboard.putNumber("OI/Arm Power", oi.getArmPower());
        SmartDashboard.putNumber("OI/K", oi.getArmPower()/Math.cos(Math.abs(arm.getRadians())));
    }
}
