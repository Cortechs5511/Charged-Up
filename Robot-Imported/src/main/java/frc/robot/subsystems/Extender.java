package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Extender extends SubsystemBase {
    private final CANSparkMax extender = createArmController(ArmConstants.EXTENDER_ID, false);

    private final RelativeEncoder extenderEncoder = createEncoder(extender);
    private double maxPower = 1.0;
    

    public Extender() {
        zero();
    }
    
    public void zero() {
        extenderEncoder.setPosition(0);
    }
    

    public double getExtenderPostion() {
        return extenderEncoder.getPosition();
    }


    public double getExtenderVelocity() {
        return extenderEncoder.getVelocity();
    }

    public void setExtendPower(double power) {
        extender.set(power*maxPower);
    }

    private CANSparkMax createArmController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(ArmConstants.VOLTAGE_COMPENSATION);
        controller.setIdleMode(ArmConstants.IDLE_MODE);
        controller.setOpenLoopRampRate(ArmConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(ArmConstants.RAMP_RATE);

        controller.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        controller.setSecondaryCurrentLimit(100);

        controller.setInverted(isInverted);
        return controller;
    }

    private RelativeEncoder createEncoder(CANSparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();

        return encoder;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Extension", getExtenderPostion());
        if (Constants.DIAGNOSTICS) {
            SmartDashboard.putNumber("Arm/Extension", getExtenderPostion());
        }
    }
}
