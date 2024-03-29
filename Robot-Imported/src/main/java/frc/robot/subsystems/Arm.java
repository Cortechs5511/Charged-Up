package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax leftLeader = createArmController(ArmConstants.LEFT_ID, false);
    // private final CANSparkMax rightLeader = createArmController(ArmConstants.RIGHT_ID, true);
    //private final CANSparkMax extender = createArmController(ArmConstants.EXTENDER_ID, false);

    private final RelativeEncoder leftEncoder = createEncoder(leftLeader);
    // private final RelativeEncoder rightEncoder = createEncoder(rightLeader);
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9);
    //private final RelativeEncoder extenderEncoder = createEncoder(extender);
    private double maxPower = 0.5;
    

    public Arm() {
        zero();
        SmartDashboard.putNumber("Arm/Maxpower", 0.5);

    }
    
    public void zero() {
        leftEncoder.setPosition(0);

    }
    
    public double getArmPosition() {
        return Math.abs(absoluteEncoder.getAbsolutePosition()-0.899-0.079);
    }

    public double getLeftPosition() {
        return (leftEncoder.getPosition()/81);
    }

    // public double getRightPosition() {
    //     return (rightEncoder.getPosition()/81);
    // }
    public double getRadians() {
        return getArmPosition()*2*Math.PI;
    }
    // public double getRightRadians() {
    //     return (2*Math.PI*getRightPosition());
    //     }

    public double getLeftRadians() {
        return (2*Math.PI*getLeftPosition());
    }

    // public double getExtenderPostion() {
    //     return extenderEncoder.getPosition();
    // }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity() ;
    }

    // public double getRightVelocity() {
    //     return rightEncoder.getVelocity();
    // }

    // public double getExtenderVelocity() {
    //     return extenderEncoder.getVelocity();
    // }

    public void setPower(double speed) {
        // rightLeader.set(speed * maxPower);
        leftLeader.set(speed * maxPower);
    }

    // public void setExtendPower(double power) {
    //     extender.set(power*maxPower);
    // }

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
        // SmartDashboard.getNumber("Arm/Maxpower", maxPower);
        SmartDashboard.putNumber("Arm/Absolute Encoder Value", getArmPosition());
       
        if (Constants.DIAGNOSTICS) {
            SmartDashboard.getNumber("Arm/Maxpower", maxPower);

            SmartDashboard.putNumber("Arm/Left Position", getLeftPosition());
            // SmartDashboard.putNumber("Arm/Right Position", getRightPosition());
            // SmartDashboard.putNumber("Arm/RighRadians", getRightRadians());
            SmartDashboard.putNumber("Arm/LeftRadians", getLeftRadians());
            SmartDashboard.putNumber("Arm/TRUE Radians", getRadians());

            SmartDashboard.putNumber("Arm/Absolute Encoder Value", getArmPosition());

            SmartDashboard.putNumber("Arm/Left Velocity", getLeftVelocity());
            // SmartDashboard.putNumber("Arm/Right Velocity", getRightVelocity());

            SmartDashboard.putNumber("Arm/Left Temp", leftLeader.getMotorTemperature());
            // SmartDashboard.putNumber("Arm/Right Temp", rightLeader.getMotorTemperature());

            SmartDashboard.putNumber("Arm/Left Current", leftLeader.getOutputCurrent());
            // SmartDashboard.putNumber("Arm/Right Current", rightLeader.getOutputCurrent());
        }
    }
}
