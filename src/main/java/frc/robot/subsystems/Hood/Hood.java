package frc.robot.subsystems.Hood;

import frc.robot.constants.Motors;
import frc.robot.constants.Settings;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    
    private TalonFX hoodMotor;
    private CANcoder hoodEncoder;

    // rotations
    private double targetRotations;
    private double targetVelRPS;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

    public Hood() {
        hoodMotor = new TalonFX(Motors.HoodConstants.HOOD_MOTOR, Settings.upper);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        hoodEncoder = new CANcoder(Motors.HoodConstants.HOOD_ENCODER, Settings.upper);

        hoodMotor.getConfigurator().apply(Motors.HoodConstants.hoodMotorConfigs.getConfiguration());
        hoodEncoder.getConfigurator().apply(Motors.HoodConstants.hoodEncoderConfigs.getConfiguration());
    }

    // Degrees to rotations
    public void setHoodAngle(double targetRot) {
        hoodMotor.setControl(positionVoltage.withPosition(targetRot));
    }

    public void stowHood() {
        targetVelRPS = Settings.Hood.HOOD_RESET_RPS;
        hoodMotor.setControl(velocityVoltage.withVelocity(targetVelRPS));
    }

    public void resetHood() {
        hoodEncoder.setPosition(0);
        targetVelRPS = 0;
    }

    public boolean hoodAtPosition() {
        return Math.abs(hoodEncoder.getPosition().getValueAsDouble() - targetRotations)
                < Settings.Hood.HOOD_TOLERANCE;
    }

    public boolean hoodIsStalling() {
        return (hoodMotor.getStatorCurrent().getValueAsDouble() > Settings.Hood.HOOD_STALL_CURRENT)
                && (targetVelRPS == Settings.Hood.HOOD_RESET_RPS);
    }

    public double getHoodAngleDegrees() {
        return Units.rotationsToDegrees(hoodEncoder.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/TargetAngleDegrees", Units.rotationsToDegrees(targetRotations));
        SmartDashboard.putNumber("Hood/CurrentAngleDegrees", getHoodAngleDegrees());
        SmartDashboard.putBoolean("Hood/isStalling", hoodIsStalling());
        SmartDashboard.putBoolean("Hood/atPosition", hoodAtPosition());
    }
}