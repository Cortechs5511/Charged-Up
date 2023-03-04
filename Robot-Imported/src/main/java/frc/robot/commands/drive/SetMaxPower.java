package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetMaxPower extends InstantCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drive drive;
    private double maxPower;

    public SetMaxPower(Drive drive, double maxPower)  {
        this.drive = drive;
        this.maxPower = maxPower;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setMaxPower(maxPower);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
