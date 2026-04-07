package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootTest3 extends Command {
    private final Shooter shooter;

    public ShooterShootTest3(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setSpeedRPS(45);
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