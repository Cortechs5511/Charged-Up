package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;




public class StartAutoAlign extends CommandBase {
    private final Drive drive;
    public StartAutoAlign(Drive subsystem) {
        drive = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (Math.abs(drive.getYaw()) < DriveConstants.ALIGN_INIT_ANGLE) {
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
        if (Math.abs(drive.getYaw()) >= DriveConstants.ALIGN_INIT_ANGLE) {
            return true;
        } else {
            return false;
        }
    }
}
