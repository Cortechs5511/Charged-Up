package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final CANSparkMax leftLeader = createArmController(ArmConstants.LEFT_ID, false);
    private final CANSparkMax rightLeader = createArmController(ArmConstants.RIGHT_ID, true);

    private final RelativeEncoder leftEncoder = createEncoder(leftLeader);
    private final RelativeEncoder rightEncoder = createEncoder(rightLeader);
    private double maxPower = 1.0;
    

    public Arm() {
        zero();
    }
    
    public void zero() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity() ;
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public void setPower(double speed) {
        rightLeader.set(speed * maxPower);
        leftLeader.set(speed * maxPower);
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
       
        if (Constants.DIAGNOSTICS) {
            SmartDashboard.putNumber("Arm/Left Position", getLeftPosition());
            SmartDashboard.putNumber("Arm/Right Position", getRightPosition());

            SmartDashboard.putNumber("Arm/Left Velocity", getLeftVelocity());
            SmartDashboard.putNumber("Arm/Right Velocity", getRightVelocity());

            SmartDashboard.putNumber("Arm/Left Temp", leftLeader.getMotorTemperature());
            SmartDashboard.putNumber("Arm/Right Temp", rightLeader.getMotorTemperature());

            SmartDashboard.putNumber("Arm/Left Current", leftLeader.getOutputCurrent());
            SmartDashboard.putNumber("Arm/Right Current", rightLeader.getOutputCurrent());
        }
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> ecc6e08d479b12293b4235537aaabe0c5175f218
