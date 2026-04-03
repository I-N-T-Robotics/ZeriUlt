package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootTest extends Command {
    private final Shooter shooter;

    public ShooterShootTest(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setSpeedRPS(20);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end (boolean interupted) {
        shooter.setSpeedRPS(0);
    }
}