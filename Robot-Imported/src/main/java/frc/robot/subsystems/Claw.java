package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private final CANSparkMax claw = createArmController(ClawConstants.CLAW_ID, false);

    private final RelativeEncoder clawEncoder = createEncoder(claw);
    private double maxPower = 1.0;
    

    public Claw() {
        zero();
    }
    
    public void zero() {
        clawEncoder.setPosition(0);
    }
    

    public double getClawPostion() {
        return clawEncoder.getPosition();
    }


    public double getClawVelocity() {
        return clawEncoder.getVelocity();
    }

    public void setClawPower(double power) {
        claw.set(power*maxPower);
    }

    private CANSparkMax createArmController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(ArmConstants.VOLTAGE_COMPENSATION);
        controller.setIdleMode(ArmConstants.IDLE_MODE);
        controller.setOpenLoopRampRate(ArmConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(ArmConstants.RAMP_RATE);

        controller.setSmartCurrentLimit(ArmConstants.CLAW_CURRENT_LIMIT);
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
        // SmartDashboard.putNumber("Claw/State", getClawPostion());
        if (Constants.DIAGNOSTICS) {
            SmartDashboard.putNumber("Claw/State", getClawPostion());
            SmartDashboard.putNumber("Claw/State", getClawPostion());
        }
    }
}
