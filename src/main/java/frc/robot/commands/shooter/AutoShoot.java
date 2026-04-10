package frc.robot.commands.shooter;

import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.util.Distance;
import frc.robot.util.ShootInterpolation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShoot extends Command {
    private final Shooter shooter;
    private final Spindexer spindexer;
     private final CommandSwerveDrivetrain drivetrain;

    public AutoShoot(Shooter shooter, Spindexer spindexer, CommandSwerveDrivetrain drivetrain) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.drivetrain = drivetrain;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d targetPose = Robot.isBlue() ? Field.BLUE_GOAL_CENTER : Field.RED_GOAL_CENTER;

        double distance = Distance.calculateDistance(robotPose.getX(), robotPose.getY(), targetPose.getX(), targetPose.getY());

    
        shooter.setSpeedRPS(ShootInterpolation.getRPM(distance));
        if (spindexer.getIsStalling()) {
            spindexer.reverseSpindexer();
            spindexer.reverseTransition();
        } else if (shooter.shooterAtSpeed()) {
            spindexer.startTransition();
            spindexer.startSpindexer();
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