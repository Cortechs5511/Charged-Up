package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Extender extends SubsystemBase {
    private final CANSparkMax extender = createArmController(ArmConstants.EXTENDER_ID, false);
    private static double stringPotOffset;

    private final RelativeEncoder extenderEncoder = createEncoder(extender);
    private double maxPower = 1.0;

    AnalogPotentiometer stringPot = new AnalogPotentiometer(ArmConstants.STRING_POT_ID, ArmConstants.STRING_POT_RANGE, 0);

    public Extender() {
        zero();
    }
    
    public void zero() {
        extenderEncoder.setPosition(0);
    }
    
    public void goToPosition(double power, double target) {
        
        if(stringPot.get() < target - 2) {
            extender.set(power*maxPower);
        }
        else if (stringPot.get() > target + 2) {
            extender.set(-power*maxPower);
        } else {
            extender.set(0);
        }
    }

    public double getExtenderEncoderPosition() {
        return extenderEncoder.getPosition();
    }

    public void setOffset(double offset) {
        stringPotOffset = offset;
    }

    public double getOffset() {
        return stringPotOffset;
    }
    public double getExtenderPosition() {
        return (getExtenderEncoderPosition() / 25) * ArmConstants.AVERAGE_PULLEY_DIAMETER * Math.PI;
    }

    public double getStringPotPosition() {
        return Math.abs(stringPot.get());
    }

    public double getExtenderVelocity() {
        return extenderEncoder.getVelocity();
    }

    public double getCurrent() {
        return extender.getOutputCurrent();
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

        controller.setSmartCurrentLimit(ArmConstants.WINCH_CURRENT_LIMIT);
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
        // SmartDashboard.putNumber("Arm/Encoder Extension Position", getExtenderEncoderPosition());
        SmartDashboard.putNumber("Arm/String Extension", getStringPotPosition());
        SmartDashboard.putNumber("Arm/Offset", getOffset());
        // SmartDashboard.putNumber("Arm/String Potentiometer", getStringPotPosition());
        // SmartDashboard.putNumber("Arm/Winch Current", getCurrent());
        if (Constants.DIAGNOSTICS) {
            SmartDashboard.putNumber("Arm/Encoder Extension Position", getExtenderEncoderPosition());
            SmartDashboard.putNumber("Arm/String Extension", getStringPotPosition());
            SmartDashboard.putNumber("Arm/Winch Current", getCurrent());
            SmartDashboard.putNumber("Arm/Encoder Extension Position", getExtenderEncoderPosition());
            SmartDashboard.putNumber("Arm/Extension", getExtenderPosition());
        }
    }
}
