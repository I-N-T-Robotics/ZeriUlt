package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.util.ShootInterpolation;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootTest extends Command {
    private final Shooter shooter;

    public ShooterShootTest(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setRightMotorRPM(ShootInterpolation.getRPM(3000));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}