package frc.robot.commands.hood;

import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.util.Distance;
import frc.robot.util.HoodInterpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodAim extends Command {
    private final Hood hood;
    private final CommandSwerveDrivetrain drivetrain;

    public HoodAim(Hood hood, CommandSwerveDrivetrain drivetrain) {
        this.hood = hood;
        this.drivetrain = drivetrain;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d targetPose = new Translation2d(); //TODO: Get from robot contianer

        double distance = Distance.calculateDistance(robotPose.getX(), robotPose.getY(), targetPose.getX(), targetPose.getY());

        hood.setHoodAngle(HoodInterpolation.getAngle(distance)); //TODO: make sure distance is in inches (For shooter too)
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}