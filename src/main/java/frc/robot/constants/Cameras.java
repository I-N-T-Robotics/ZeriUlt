package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface Cameras {
    
    public Camera[] LimelightCameras = new Camera[] {
        new Camera("limelight-turret", new Pose3d(-6.771, -8.047, 13.13, new Rotation3d(0, 34, 135))), //TODO: y component is super wrong??
        new Camera("limelight-right", new Pose3d(-1.5, 12.828, 4.692, new Rotation3d(0, 34, -90))) //10.96.42.12:5801
    };

    public static class Camera {
        private String name;
        private Pose3d location;

        public Camera(String name, Pose3d location) {
            this.name = name;
            this.location = location;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }
    }
}