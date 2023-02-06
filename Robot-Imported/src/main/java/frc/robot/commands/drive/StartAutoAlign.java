package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class StartAutoAlign extends CommandBase {
    private final Drive drive;
    Timer timer = new Timer();
    public StartAutoAlign(Drive subsystem) {
        drive = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        if (Math.abs(drive.getPitch()) < 18 && timer.hasElapsed(3.0)) {
            drive.setPower(-0.35, -0.35);          
        } else {
        drive.setPower(0,0); }

    }
    @Override
    public void end(boolean interrupted) {
        drive.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(drive.getPitch()) >= 18) {
            return true;
        } else {
            return false;
        }
    }
}
