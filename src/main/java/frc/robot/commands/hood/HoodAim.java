package frc.robot.commands.hood;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.util.Distance;
import frc.robot.util.HoodInterpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodAim extends Command {
    private final Hood hood;
    private final CommandSwerveDrivetrain drivetrain;
    private final RobotContainer robotContainer;
    private boolean isFerrying = false;

    public HoodAim(Hood hood, CommandSwerveDrivetrain drivetrain, RobotContainer robotContainer) {
        this.hood = hood;
        this.drivetrain = drivetrain;
        this.robotContainer = robotContainer;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d targetPose = robotContainer.getGoalPosition();

        double distance = Distance.calculateDistance(robotPose.getX(), robotPose.getY(), targetPose.getX(), targetPose.getY());

        if (getIsFerrying()) {
            hood.setHoodAngle(0.9);
        } else {
            hood.setHoodAngle(0.2);
            // hood.setHoodAngle(HoodInterpolation.getAngle(distance));
        }
    }

  public boolean robotIsOnAllianceSide() {
    Pose2d pose = drivetrain.getPose();
    return Robot.isBlue()
        ? pose.getX() < Units.inchesToMeters(182.11 + 12)
        : pose.getX() > Units.inchesToMeters(469.11 - 12);
  }

  public boolean getIsFerrying() {
    isFerrying = !robotIsOnAllianceSide();
    return isFerrying;
  }

    @Override
    public boolean isFinished() {
        return false;
    }
}