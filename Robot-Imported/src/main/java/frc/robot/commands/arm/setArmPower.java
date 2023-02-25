package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;
import frc.robot.OI;
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
        arm.setPower(oi.getArmPower());
       //drive.setPower(oi.getLeftYDeadband(), oi.getRightYDeadband());
    }
}