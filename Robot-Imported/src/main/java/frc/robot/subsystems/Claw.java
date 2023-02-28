package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
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

    // public void gripperToggle() {
    //   gripperSolenoid.toggle();
    //   gripperState = !gripperState;
    // }

    // public void extenderToggle() {
    //   extenderSolenoid.toggle();
    //   extenderState = !extenderState;
    // }

    public void setGripper(boolean channel) {
      if (channel) {
        gripperSolenoid.set(kForward);
        gripperState = true;
      }
      else {
        gripperSolenoid.set(kReverse);
        gripperState = false;
      }
    }

    public void setExtender(boolean channel) {
      if (channel) {
        extenderSolenoid.set(kForward);
        extenderState = true;
      }
      else {
        extenderSolenoid.set(kReverse);
        extenderState = false;
      }
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