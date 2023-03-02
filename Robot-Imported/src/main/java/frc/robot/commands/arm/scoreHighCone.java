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
        while(arm.getRotations() < ArmConstants.HIGH_CONE_ROTATIONS) {
            arm.setPower(0.5);
        }
    }
}
