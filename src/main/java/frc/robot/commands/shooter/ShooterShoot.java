package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.Distance;
import frc.robot.util.ShootInterpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShoot extends Command {
    private final Shooter shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;

    public ShooterShoot(Shooter shooter, CommandSwerveDrivetrain drivetrain, Turret turret) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.turret = turret;
        addRequirements(shooter, turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d targetPose = new Translation2d(); //TODO: Get from robot contianer

        double distance = Distance.calculateDistance(robotPose.getX(), robotPose.getY(), targetPose.getX(), targetPose.getY());

        shooter.setSpeedRPS(ShootInterpolation.getRPM(distance));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}