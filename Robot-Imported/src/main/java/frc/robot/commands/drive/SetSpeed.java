package frc.robot.commands.drive;

import frc.robot.OI;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSpeed extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drive drive;
    private final OI oi = OI.getInstance();

    public SetSpeed(Drive subsystem) {
        drive = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // drive.arcadeDrive(oi.getLeftYDeadband(), oi.getRightXDeadband());
        //drive.setPower(oi.getLeftYDeadband(), oi.getRightYDeadband());

       drive.tankDrive(oi.getLeftYDeadband(), oi.getRightYDeadband());

        //speed control, untested, and we need to find out max velocity
       //drive.speedDrive(DriveConstants.MAX_VELOCITY*oi.getLeftYDeadband(), DriveConstants.MAX_VELOCITY*oi.getRightYDeadband());

    }

    @Override
    public void end(boolean interrupted) {
        drive.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
