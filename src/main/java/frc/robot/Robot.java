package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.vision.LimelightHelpers;

public class Robot extends TimedRobot {

    public static boolean isBlue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.get() == Alliance.Blue;
    }

    //TODO: make sure static works here
    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    private RobotContainer robot;
    private Command auto;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        robot.AddVisionMeasurement().schedule();
        CommandScheduler.getInstance().run();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        robot.getLimelightVision().setMegaTag2(false);
        LimelightHelpers.SetIMUMode("limelight-turret", 1);
    }

    @Override
    public void disabledPeriodic() {
         SmartDashboard.putNumber("Hub pose x", robot.getGoalPosition().getX());
        SmartDashboard.putNumber("Hub pose y", robot.getGoalPosition().getY());
        SmartDashboard.putBoolean("Alliance color blue?", isBlue());
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();
        robot.getLimelightVision().setMegaTag2(false);
        LimelightHelpers.SetIMUMode("limelight-turret", 3);

        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        robot.getLimelightVision().setMegaTag2(false);
        LimelightHelpers.SetIMUMode("limelight-turret", 3);

        if (auto != null) {
            auto.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Hub pose x", robot.getGoalPosition().getX());
        SmartDashboard.putNumber("Hub pose y", robot.getGoalPosition().getY());
        SmartDashboard.putBoolean("Alliance color blue?",isBlue());
    }

    @Override
    public void teleopExit() {
    }

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
