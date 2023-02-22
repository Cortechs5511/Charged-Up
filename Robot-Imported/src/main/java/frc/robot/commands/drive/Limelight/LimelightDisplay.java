package frc.robot.commands.drive.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimelightDisplay extends CommandBase {
    private final Limelight limelight;
    public LimelightDisplay(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(this.limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Limelight/Latency", limelight.getLatency());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}