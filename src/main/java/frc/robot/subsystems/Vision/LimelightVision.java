package frc.robot.subsystems.Vision;

// import frc.robot.Robot;
// import frc.robot.constants.Cameras;
// import frc.robot.constants.Cameras.Camera;
// import frc.robot.constants.Settings;
// import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
// import frc.robot.util.vision.LimelightHelpers;
// import frc.robot.util.vision.LimelightHelpers.PoseEstimate;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class LimelightVision extends SubsystemBase {

//     private CommandSwerveDrivetrain drivetrain;

//     public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;
//     }

//     public enum MegaTagMode {
//         MT1,
//         MT2
//     }

//     private MegaTagMode megaTagMode;
//     private int imuMode;
//     private int maxTagCount;

//     public LimelightVision(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;
        
//         for (Camera camera : Cameras.LimelightCameras) {
//             Pose3d cameraLocation = camera.getLocation();
//             LimelightHelpers.setCameraPose_RobotSpace(
//                 camera.getName(),
//                 cameraLocation.getX(),
//                 -cameraLocation.getY(),
//                 cameraLocation.getZ(),
//                 Units.radiansToDegrees(cameraLocation.getRotation().getX()),
//                 Units.radiansToDegrees(cameraLocation.getRotation().getY()),
//                 Units.radiansToDegrees(cameraLocation.getRotation().getZ())
//                 );
//         }
//         setMegaTagMode(MegaTagMode.MT1);
//         setIMUMode(1);
//         maxTagCount = 0;
//     }

//     public void setMegaTagMode(MegaTagMode mode) {
//         this.megaTagMode = mode;
//         switch (mode) {
//             case MT1:
//                 drivetrain.setVisionMeasurementStdDevs(Settings.Vision.MT1_STDDEVS);
//                 break;
//             case MT2:
//                 drivetrain.setVisionMeasurementStdDevs(Settings.Vision.MT1_STDDEVS);
//                 break;
//         }
//     }
    
//     public void setPipelineMode(int pipeline, String limelightName) {
//         LimelightHelpers.setPipelineIndex(limelightName, pipeline);
//     }

//     public void setIMUMode(int mode) {
//         this.imuMode = mode;
//         for (Camera camera : Cameras.LimelightCameras) {
//             LimelightHelpers.SetIMUMode(camera.getName(), mode);
//         }
//     }

//     public int getMaxTagCount() {
//         return this.maxTagCount;
//     }

//     public MegaTagMode getMegaTagMode() {
//         return megaTagMode;
//     }

//     public PoseEstimate getMT1PoseEstimate(String limelightName) {
//         return Robot.isBlue()
//             ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
//             : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
//     }

//     public PoseEstimate getMT2PoseEstimate(String limelightName) {
//         return Robot.isBlue()
//             ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
//             : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
//     }

//     @Override
//     public void periodic() {
//         this.maxTagCount = 0;

//         for (Camera camera : Cameras.LimelightCameras) {
//             PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MT2)
//                 ? getMT2PoseEstimate(camera.getName())
//                 : getMT1PoseEstimate(camera.getName());

//             LimelightHelpers.SetRobotOrientation(
//                 camera.getName(),
//                 drivetrain.getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180) % 360,
//                 imuMode, imuMode, imuMode, imuMode, imuMode);

//                 if ((poseEstimate != null) && poseEstimate.tagCount > 0) {
//                     drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
//                     SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", true);
//                     SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", poseEstimate.tagCount);
//                     maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
//                 } else {
//                     SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", false);
//                     SmartDashboard.putNumber("Vision/" + camera.getName() + "/TagCount", 0);
//                 }   
//         }
//         SmartDashboard.putString("Vision/Megatag Mode", getMegaTagMode().toString());
//         SmartDashboard.putNumber("Vision/IMU Mode", imuMode);
//     }
// }

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Cameras;
import frc.robot.constants.Settings;
import frc.robot.util.vision.LimelightHelpers;
import frc.robot.util.vision.LimelightHelpers.PoseEstimate;

import java.util.Optional;

public class LimelightVision extends SubsystemBase {

  // Dynamically sized to match however many cameras are in Cameras.LimelightCameras
  private PoseEstimate[] lastEstimates = new PoseEstimate[Cameras.LimelightCameras.length];
  private boolean[] estimateUpdated = new boolean[Cameras.LimelightCameras.length];

  public boolean useMT2 = false;

  public LimelightVision() {
    for (int i = 0; i < lastEstimates.length; i++) {
      lastEstimates[i] = new PoseEstimate();
      estimateUpdated[i] = false;
    }
  }

  public void setMegaTag2(boolean useMT2) {
    this.useMT2 = useMT2;
  }

  public boolean getMegaTag2() {
    return useMT2;
  }

  public Matrix<N3, N1> getSTD() {
    return useMT2 ? Settings.Vision.MT2_STDDEVS : Settings.Vision.MT1_STDDEVS;
  }

  public double[] getDeviations() {
    return getSTD().getData();
  }

  // Uses the first camera in LimelightCameras as the primary target camera
  public double[] getTargetMetrics() {
    return LimelightHelpers.getT2DArray(Cameras.LimelightCameras[0].getName());
  }

  public double getRange() {
    return LimelightHelpers.getTY(Cameras.LimelightCameras[0].getName());
  }

  public double getYaw() {
    return LimelightHelpers.getTX(Cameras.LimelightCameras[0].getName());
  }

  public PoseEstimate[] getPoseEstimates() {
    return lastEstimates;
  }

  public boolean rejectUpdate(PoseEstimate estimate, AngularVelocity angularVelocity) {
    if (angularVelocity.compareTo(Settings.Vision.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    if (estimate.tagCount == 0) {
      return true;
    }

    if (estimate.tagCount == 1 && estimate.avgTagArea > Settings.Vision.AREA_THRESHOLD) {
      return false;
    } else if (estimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  public void setCurrentEstimates(AngularVelocity currentAngularVelocity) {
    for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
      String name = Cameras.LimelightCameras[i].getName();

      PoseEstimate estimate = useMT2
          ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
          : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

      if (estimate != null && !rejectUpdate(estimate, currentAngularVelocity)) {
        lastEstimates[i] = estimate;
        estimateUpdated[i] = true;
      }
    }
  }

  public Optional<PoseEstimate> updatePoseEstimates(AngularVelocity currentAngularVelocity) {
    setCurrentEstimates(currentAngularVelocity);

    // Among all updated estimates, return the one with the smallest average tag distance
    PoseEstimate best = null;
    for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
      if (estimateUpdated[i]) {
        if (best == null || lastEstimates[i].avgTagDist < best.avgTagDist) {
          best = lastEstimates[i];
        }
        estimateUpdated[i] = false;
      }
    }

    return Optional.ofNullable(best);
  }
}