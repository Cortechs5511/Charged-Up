package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class goToSubstation extends CommandBase {
    private final Drive drive;
    private final Claw claw;
    
    public goToSubstation(Drive drive, Claw claw) {
        this.claw = claw;
        this.drive = drive;
        addRequirements(claw);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setClawPower(-1);
        drive.setPower(-0.1, -0.1);
    }

    @Override
    public void end(boolean interrupted) {
        claw.setClawPower(0);
        drive.setPower(0, 0);
    }

    public boolean isFinished() {
        return false;
    }
}
