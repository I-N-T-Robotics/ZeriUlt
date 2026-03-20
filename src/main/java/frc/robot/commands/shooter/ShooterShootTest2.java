package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.util.ShootInterpolation;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootTest2 extends Command {
    private final Shooter shooter;

    public ShooterShootTest2(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setRightMotorRPM(4000);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end (boolean interupted) {
        shooter.setRightMotorRPM(0);
    }
}