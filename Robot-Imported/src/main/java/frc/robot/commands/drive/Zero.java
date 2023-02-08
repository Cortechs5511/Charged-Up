package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Zero extends CommandBase {
    private Drive drive;

    public Zero(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.zero();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
