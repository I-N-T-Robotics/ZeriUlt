package frc.robot.subsystems.Shooter;

import frc.robot.constants.Motors;
import frc.robot.constants.Motors.ShooterConstants;
import frc.robot.constants.Settings;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX rightMotor;
    private TalonFX leftMotor;

    // Stored in RPS (TalonFX native)
    private double targetRPS;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    public Shooter() {
        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR, Settings.upper);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR, Settings.upper);
        leftMotor.setNeutralMode(NeutralModeValue.Coast);

        leftMotor.setControl(new Follower(Motors.ShooterConstants.RIGHT_MOTOR, MotorAlignmentValue.Opposed));

        rightMotor.getConfigurator().apply(Motors.ShooterConstants.shooterRightMotorConfig.getConfiguration());
        leftMotor.getConfigurator().apply(Motors.ShooterConstants.shooterLeftMotorConfig.getConfiguration());
    }

    public void setSpeedRPS(double rps) {
        targetRPS = rps;
    }

    public double getRightMotorRPS() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    public double getLeftMotorRPS() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    public void stopShooter() {
        targetRPS = 0;
    }

    public double getRightTargetVoltage() {
        return rightMotor.getMotorVoltage().getValueAsDouble();
    }

    public boolean shooterAtSpeed() {
        return (Math.abs(getRightMotorRPS() - targetRPS) < Settings.Shooter.SHOOTER_RPS_TOLERANCE) &&
               (Math.abs(getLeftMotorRPS() - targetRPS) < Settings.Shooter.SHOOTER_RPS_TOLERANCE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/RightRPS", getRightMotorRPS());
        SmartDashboard.putNumber("Shooter/LeftRPS", getLeftMotorRPS());
        SmartDashboard.putNumber("Shooter/TargetRPS", targetRPS);
        SmartDashboard.putBoolean("Shooter/atSpeed", shooterAtSpeed());
        SmartDashboard.putNumber("Shooter/RightTargetVoltage", getRightTargetVoltage());
        
        rightMotor.setControl(velocityRequest.withVelocity(targetRPS));
    }
}