package frc.robot.subsystems.Turret;

import java.util.Optional;

import frc.robot.constants.Motors.TurretConstants;
import frc.robot.constants.Settings;
import frc.robot.util.ChineseRemainderTheorem;
import frc.robot.util.SysId;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Turret extends SubsystemBase {

    private TalonFX turretMotor;
    private CANcoder turretMotorEncoderTurret; // 60T
    private CANcoder turretMotorEncoderEncoder; // 43T

    private PositionVoltage targetPosition;
    private double targetPositionRotations;

    private boolean aimingAtGoal = true;
    private boolean isSeeded = false;
    private boolean isFerrying = false;

    // private static final Turret instance;

    // static {
    //     instance = new Turret();
    // }

    // public static Turret getInstance() {
    //     return instance;
    // }

    public Turret() {
        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR, Settings.upper);
        turretMotor.getConfigurator();
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        turretMotorEncoderTurret = new CANcoder(TurretConstants.TURRET_ENCODER_TURRET, Settings.upper);

        turretMotorEncoderEncoder = new CANcoder(TurretConstants.TURRET_ENCODER_ENCODER, Settings.upper);

        targetPosition = new PositionVoltage(0).withEnableFOC(true);

        Optional.empty();
    }

    public double getAbsoluteTurretRotations() {
        double encoderEncoder = turretMotorEncoderEncoder.getPosition().getValueAsDouble();
        double turretEncoder = turretMotorEncoderTurret.getPosition().getValueAsDouble();

        return ChineseRemainderTheorem.getTurretRotations(
                turretEncoder,
                encoderEncoder,
                60,
                43,
                136);
    } // returns rotations

    public Rotation2d getAbsoluteTurretRotationsRot2d() {
        return Rotation2d.fromRotations(getAbsoluteTurretRotations());
    } // returns rotation2d

    public void reset() {
        turretMotorEncoderTurret.getConfigurator().setPosition(0);
        turretMotorEncoderEncoder.getConfigurator().setPosition(0);
        setCurrentPosition();
    }

    public void setCurrentPosition() {
        double motorRotations = getAbsoluteTurretRotations() * Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
        turretMotor.getConfigurator().setPosition(motorRotations);
    }
  

    public boolean atTarget() {
        return Math.abs(getTurretRotations() - targetPositionRotations) < (Units.radiansToDegrees(Settings.Turret.Constants.toleranceRadians) / 360.0);
    }

    public void setTarget(double rot) {
        targetPositionRotations = MathUtil.clamp(rot, Settings.Turret.Constants.TURRET_MIN_ROTATIONS, Settings.Turret.Constants.TURRET_MAX_ROTATIONS);
    }

    public double getTurretRotations() {
        return turretMotor.getPosition().getValueAsDouble() / Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
    }

    public double getMotorRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Turret/atTarget", atTarget());
        SmartDashboard.putNumber("Turret/absPositionCRT", getAbsoluteTurretRotations());
        SmartDashboard.putNumber("Turret/motorPosition", getMotorRotations());
        SmartDashboard.putNumber("Turret/turretPosition", getTurretRotations());
        SmartDashboard.putNumber("Turret/targetTurretPosition", targetPositionRotations);

        if (!isSeeded && Math.abs(turretMotor.getRotorVelocity().getValueAsDouble()) < 1) {
            setCurrentPosition();
            isSeeded = true;
        }

        if (isSeeded) {
            double targetMotorRots = targetPositionRotations * Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
            turretMotor.setControl(targetPosition.withPosition(targetMotorRots));
        }
    }   

    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> this.turretMotor.getPosition().getValueAsDouble(),
                () -> this.turretMotor.getVelocity().getValueAsDouble(),
                () -> this.turretMotor.getMotorVoltage().getValueAsDouble(),
                this);
    }

    private void setVoltageOverride(Optional<Double> volts) {
    }
}