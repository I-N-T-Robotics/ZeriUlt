package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class ResetTurret extends Command {
    private final Turret turret;

    public ResetTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        turret.driveToHardstop();
    }

    @Override
    public boolean isFinished() {
        return turret.isAtHardstop();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            // Only zero if we actually hit the hardstop, not if command was cancelled
            turret.resetToZero();
        }
    }
}