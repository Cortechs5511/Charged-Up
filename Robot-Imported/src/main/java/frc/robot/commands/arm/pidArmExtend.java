package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Extender;

public class pidArmExtend extends CommandBase {
    private final Extender extender;
    private double setpoint;
    private double output;
    private PIDController rotateController = new PIDController(
        ArmConstants.EXTEND_Kp, 0, ArmConstants.EXTEND_Kd);

    public pidArmExtend(Extender extender, Double setpoint) {
        this.extender = extender;
        this.setpoint = setpoint;
        addRequirements(extender);
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Arm/Extender setpoint", setpoint);
    }

    @Override
    public void execute(){
        double position = extender.getExtenderPostion();
        output = rotateController.calculate(position, setpoint);
        extender.setExtendPower(output);
    }

    @Override 
    public void end(boolean interrupted) {
        extender.setExtendPower(0);
    }

    public boolean isFinished(){
        return Math.abs(output) < 0.05;
    }

}
