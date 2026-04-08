package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.Distance;
import frc.robot.util.ShootInterpolation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShoot extends Command {
    private final Shooter shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Supplier<Translation2d> targetPoseSupplier;

    public ShooterShoot(Shooter shooter, CommandSwerveDrivetrain drivetrain, Turret turret, Supplier<Translation2d> targetPoseSupplier) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d targetPose = targetPoseSupplier.get();

        double distance = Distance.calculateDistance(robotPose.getX(), robotPose.getY(), targetPose.getX(), targetPose.getY());

        if (turret.getTargetPositionRotations() > 0.7 ||
            turret.getTargetPositionRotations() < 0.0) {
                return;
            } else {
                shooter.setSpeedRPS(ShootInterpolation.getRPM(distance));
            }
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