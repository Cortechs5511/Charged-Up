package main.java.frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Claw extends SubsystemBase{
    Compressor pcmCompressor  = new Compressor(PneumaticsConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM);
    DoubleSolenoid gripperSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM,
      0, 1);
    DoubleSolenoid extenderSolenoid = new DoubleSolenoid(PneumaticsConstants.PNEUMATICS_MODULE_ID, PneumaticsModuleType.CTREPCM,
      2, 3);
    
    boolean extenderState;
    boolean gripperState;

    boolean compressorEnabled;
    boolean pressureSwitch;
    double compressorCurrent;

    public Claw() {
        pcmCompressor.enableDigital();
        gripperSolenoid.set(kReverse);
        extenderSolenoid.set(kReverse);
        extenderState = false;
        gripperState = false;
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    compressorEnabled = pcmCompressor.isEnabled();
    pressureSwitch = pcmCompressor.getPressureSwitchValue();
    compressorCurrent = pcmCompressor.getCurrent();

    SmartDashboard.putBoolean("Compressor Enabled", compressorEnabled);
    SmartDashboard.putBoolean("Pressure Switch", pressureSwitch);
    SmartDashboard.putNumber("Compressor Current", compressorCurrent);
    SmartDashboard.putBoolean("Gripper State", gripperState);
    SmartDashboard.putBoolean("Extender State", extenderState);
    
    }
}
