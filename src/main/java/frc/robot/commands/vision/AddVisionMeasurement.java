package frc.robot.commands.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Cameras;
import frc.robot.subsystems.Vision.LimelightVision;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.util.vision.LimelightHelpers;
import frc.robot.util.vision.LimelightHelpers.PoseEstimate;

import java.util.Optional;

public class AddVisionMeasurement extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightVision vision;

  public PoseEstimate estimate;
  private Matrix<N3, N1> stdDev;

  public AddVisionMeasurement(CommandSwerveDrivetrain drivetrain, LimelightVision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    addRequirements(vision);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double heading = drivetrain.getPose().getRotation().getDegrees();

    for (Cameras.Camera camera : Cameras.LimelightCameras) {
      LimelightHelpers.SetRobotOrientation(
          camera.getName(), heading, 0, 0, 0, 0, 0);
    }

    AngularVelocity currentAngularVelocity = drivetrain.getAngularVelocity();

    stdDev = vision.getSTD();

    Optional<PoseEstimate> estimatedPose = vision.updatePoseEstimates(currentAngularVelocity);

    if (estimatedPose.isPresent()) {
      drivetrain.addVisionMeasurement(
          estimatedPose.get().pose, estimatedPose.get().timestampSeconds, stdDev);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}