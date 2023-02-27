package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class pidArmRotate extends CommandBase {
    private final Arm arm;
    private double setpoint;
    private double output;
    private PIDController rotateController = new PIDController(
        ArmConstants.ROTATE_Kp, 0, ArmConstants.ROTATE_Kd);

    public pidArmRotate(Arm arm, Double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Arm/Rotate setpoint", setpoint);

    }

    @Override
    public void execute(){
        double position = arm.getLeftPosition();
        output = rotateController.calculate(position, setpoint);
        arm.setPower(output);
    }

    @Override 
    public void end(boolean interrupted) {
        arm.setPower(0);
    }

    public boolean isFinished(){
        return Math.abs(output) < 0.02;
    }

}
