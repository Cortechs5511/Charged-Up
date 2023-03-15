package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveForTime extends CommandBase {
    private final Drive drive;
    private final double time;
    private final double power;
    private final Timer timer = new Timer();

    public DriveForTime(Drive drive, double time, double power) {
        this.drive = drive;
        this.time = time;
        this.power = power;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    timer.reset();
    }

    @Override
    public void execute() {
    timer.reset();
    timer.start();
    if (!timer.hasElapsed(time)) {
        drive.setPower(power, power);
    } else {
        drive.setPower(0,0);
    }
    }

    @Override
    public void end(boolean interrupted) {
    drive.setPower(0,0);
    }

    @Override
    public boolean isFinished(){
    return(timer.hasElapsed(time));
    }
}
