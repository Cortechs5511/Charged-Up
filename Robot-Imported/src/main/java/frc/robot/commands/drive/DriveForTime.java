package frc.robot.commands.drive;

import frc.robot.OI;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForTime extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drive drive;
    private final Timer timer = new Timer();
    private final double time;
    private final OI oi = OI.getInstance();

    public DriveForTime(Drive drive, double time) {
        this.drive = drive;
        this.time = time;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(time)) {
            drive.setPower(-0.2, -0.2);
        } else {
            drive.setPower(0,0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
