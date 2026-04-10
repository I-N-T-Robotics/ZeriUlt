
// package frc.robot.commands.turret;

// import java.util.function.Supplier;

// import frc.robot.Robot;
// import frc.robot.constants.Settings;
// import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Turret.Turret;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoAim extends Command {
//   private final Turret turret;
//   private final CommandSwerveDrivetrain drive;
//   private final Supplier<Translation2d> targetPoseSupplier;
//   private boolean isFerrying = false;

//     private static final double SHOOTER_OFFSET_ROTATIONS = 0.25;// + 0.0417; // TODO: FIND

//   // Tracks which hardstop we last committed to when in the deadzone
//   private boolean lastCommittedToMax = false;

//   public AutoAim(Turret turret, CommandSwerveDrivetrain drive, Supplier<Translation2d> targetPoseSupplier) {
//     this.turret = turret;
//     this.drive = drive;
//     this.targetPoseSupplier = targetPoseSupplier;
//     addRequirements(turret);
//   }

//   @Override
//   public void execute() {
//     Pose2d robotPose = drive.getPose();
//     Translation2d targetPose = targetPoseSupplier.get();

//     double dx = targetPose.getX() - (robotPose.getX() - Units.inchesToMeters(5.65));
//     double dy = targetPose.getY() - (robotPose.getY() - Units.inchesToMeters(4.65));

//     // if (getIsFerrying()) {
//     //   double vx = drive.getChassisSpeeds().vxMetersPerSecond;
//     //   double shootWhileMovingFactor = 0.2;
//     //   dx += vx * shootWhileMovingFactor;
//     // }

//     // Rotation2d fieldAngleToTarget = new Rotation2d(dx, dy);

//     // double fieldRotations = fieldAngleToTarget.getRotations();
//     double fieldRotations = Math.atan2(dy, dx);
//     double robotRotations = robotPose.getRotation().getRotations();

//     // Wrap in full 0-1 space so inputModulus works correctly for a full circle.
//     // findBestReachableTarget handles mapping into the actual turret range.
//     double robotRelativeRotations = MathUtil.inputModulus(
//         fieldRotations + robotRotations, 0.0, 1.0);

//     double compensatedTarget = robotRelativeRotations + SHOOTER_OFFSET_ROTATIONS;

//     double currentRotations = turret.getAbsoluteTurretRotations();
//     double finalTarget = findBestReachableTarget(compensatedTarget, currentRotations);

//     turret.setTarget(finalTarget);

//     // Debug
//     SmartDashboard.putNumber("Turret/fieldAngle", fieldRotations);
//     SmartDashboard.putNumber("Turret/robotRelative", robotRelativeRotations);
//     SmartDashboard.putNumber("Turret/compensatedTarget", compensatedTarget);
//     SmartDashboard.putNumber("Turret/finalTarget", finalTarget);
//   }

//   public boolean robotIsOnAllianceSide() {
//     Pose2d pose = drive.getPose();
//     return Robot.isBlue()
//         ? pose.getX() < Units.inchesToMeters(182.11)
//         : pose.getX() > Units.inchesToMeters(469.11);
//   }

//   public boolean getIsFerrying() {
//     isFerrying = !robotIsOnAllianceSide();
//     return isFerrying;
//   }

//   private double findBestReachableTarget(double desiredTarget, double current) {
//     double min = Settings.Turret.Constants.TURRET_MIN_ROTATIONS;
//     double max = Settings.Turret.Constants.TURRET_MAX_ROTATIONS;

//     double[] candidates = {
//         desiredTarget,
//         desiredTarget + 1.0,
//         desiredTarget - 1.0
//     };

//     double bestTarget = Double.NaN;
//     double shortestDistance = Double.MAX_VALUE;

//     for (double candidate : candidates) {
//       if (candidate >= min && candidate <= max) {
//         double distance = Math.abs(candidate - current);
//         if (distance < shortestDistance) {
//           shortestDistance = distance;
//           bestTarget = candidate;
//         }
//       }
//     }

//     if (!Double.isNaN(bestTarget)) {
//       return bestTarget;
//     }

//     // Target is in the deadzone — go to whichever hardstop is angularly
//     // closest to where we want to point, accounting for wraparound
//     double distToMin = Math.min(
//         Math.abs(desiredTarget - min),
//         1.0 - Math.abs(desiredTarget - min));
//     double distToMax = Math.min(
//         Math.abs(desiredTarget - max),
//         1.0 - Math.abs(desiredTarget - max));

//     return distToMin < distToMax ? min : max;
//   }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
package frc.robot.commands.turret;

import java.util.function.Supplier;

import frc.robot.Robot;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {
  private final Turret turret;
  private final CommandSwerveDrivetrain drive;
  private final Supplier<Translation2d> targetPoseSupplier;
  private boolean isFerrying = false;

    private static final double SHOOTER_OFFSET_ROTATIONS = .23; // TODO: FIND

  // Tracks which hardstop we last committed to when in the deadzone
  private boolean lastCommittedToMax = false;

  public AutoAim(Turret turret, CommandSwerveDrivetrain drive, Supplier<Translation2d> targetPoseSupplier) {
    this.turret = turret;
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Translation2d targetPose = targetPoseSupplier.get();

    Translation2d turretFieldPosition = robotPose.getTranslation()
        .plus(Settings.Turret.Constants.TURRET_OFFSET.rotateBy(robotPose.getRotation()));

    Translation2d turretToTarget = targetPose.minus(turretFieldPosition);
    Rotation2d fieldAngleToTarget = turretToTarget.getAngle();

    Rotation2d robotRelativeAngle = fieldAngleToTarget.plus(robotPose.getRotation());
 
    double robotRelativeRotations = robotRelativeAngle.getRotations() + SHOOTER_OFFSET_ROTATIONS;

    double currentRotations = turret.getAbsoluteTurretRotations();
    double finalTarget = findBestReachableTarget(robotRelativeRotations, currentRotations);

    //  turret.setTarget(0.67);
    // turret.setTarget(finalTarget);

    // Debug
    SmartDashboard.putNumber("Turret/fieldAngleDeg", fieldAngleToTarget.getDegrees());
    SmartDashboard.putNumber("Turret/robotRelativeDeg", robotRelativeAngle.getDegrees());
    SmartDashboard.putNumber("Turret/finalTargetRot", finalTarget);
    SmartDashboard.putNumber("Turret/distanceMeters", turretToTarget.getNorm());
  }

  public boolean robotIsOnAllianceSide() {
    Pose2d pose = drive.getPose();
    return Robot.isBlue()
        ? pose.getX() < Units.inchesToMeters(182.11)
        : pose.getX() > Units.inchesToMeters(469.11);
  }

  public boolean getIsFerrying() {
    isFerrying = !robotIsOnAllianceSide();
    return isFerrying;
  }

  private double findBestReachableTarget(double desiredTarget, double current) {
    double min = Settings.Turret.Constants.TURRET_MIN_ROTATIONS;
    double max = Settings.Turret.Constants.TURRET_MAX_ROTATIONS;

    double[] candidates = {
        desiredTarget,
        desiredTarget + 1.0,
        desiredTarget - 1.0
    };

    double bestTarget = Double.NaN;
    double shortestDistance = Double.MAX_VALUE;

    for (double candidate : candidates) {
      if (candidate >= min && candidate <= max) {
        double distance = Math.abs(candidate - current);
        if (distance < shortestDistance) {
          shortestDistance = distance;
          bestTarget = candidate;
        }
      }
    }

    if (!Double.isNaN(bestTarget)) {
      return bestTarget;
    }

    // Target is in the deadzone — go to whichever hardstop is angularly
    // closest to where we want to point, accounting for wraparound
    double distToMin = Math.min(
        Math.abs(desiredTarget - min),
        1.0 - Math.abs(desiredTarget - min));
    double distToMax = Math.min(
        Math.abs(desiredTarget - max),
        1.0 - Math.abs(desiredTarget - max));

    return distToMin < distToMax ? min : max;
  }

    @Override
    public boolean isFinished() {
        return false;
    }
}
