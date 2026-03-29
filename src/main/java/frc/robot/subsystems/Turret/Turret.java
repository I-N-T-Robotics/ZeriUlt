// package frc.robot.subsystems.Turret;

// import java.util.Optional;

// import frc.robot.constants.Motors.TurretConstants;
// import frc.robot.constants.Settings;
// import frc.robot.util.ChineseRemainderTheorem;
// import frc.robot.util.SysId;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// public class Turret extends SubsystemBase {

//     private TalonFX turretMotor;
//     private CANcoder turretMotorEncoderTurret; // 60T
//     private CANcoder turretMotorEncoderEncoder; // 43T

//     private PositionVoltage targetPosition;
//     private double targetPositionRotations;

//     private boolean isSeeded = false;
//     private boolean isFerrying = false;

//     // private static final Turret instance;

//     // static {
//     //     instance = new Turret();
//     // }

//     // public static Turret getInstance() {
//     //     return instance;
//     // }

//     public Turret() {
//         turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR, Settings.upper);
//         turretMotor.getConfigurator();
//         turretMotor.setNeutralMode(NeutralModeValue.Brake);

//         turretMotorEncoderTurret = new CANcoder(TurretConstants.TURRET_ENCODER_TURRET, Settings.upper);

//         turretMotorEncoderEncoder = new CANcoder(TurretConstants.TURRET_ENCODER_ENCODER, Settings.upper);

//         targetPosition = new PositionVoltage(0).withEnableFOC(true);

//         turretMotor.getConfigurator().apply(TurretConstants.turretConfigs.getConfiguration());
//         turretMotorEncoderTurret.getConfigurator().apply(TurretConstants.turretEncoderTurret.getConfiguration());
//         turretMotorEncoderEncoder.getConfigurator().apply(TurretConstants.turretEncoderEncoder.getConfiguration());

//         isFerrying = false;

//         Optional.empty();
//     }

//     public double getAbsoluteTurretRotations() {
//         double encoderEncoder = turretMotorEncoderEncoder.getPosition().getValueAsDouble();
//         double turretEncoder = turretMotorEncoderTurret.getPosition().getValueAsDouble();

//         return ChineseRemainderTheorem.getTurretRotations(
//                 turretEncoder,
//                 encoderEncoder,
//                 60,
//                 43,
//                 136);
//     } // returns rotations

//     public Rotation2d getAbsoluteTurretRotationsRot2d() {
//         return Rotation2d.fromRotations(getAbsoluteTurretRotations());
//     } // returns rotation2d

//     public void reset() {
//         turretMotorEncoderTurret.getConfigurator().setPosition(0);
//         turretMotorEncoderEncoder.getConfigurator().setPosition(0);
//         setCurrentPosition();
//     }

//     public void setCurrentPosition() {
//         double motorRotations = getAbsoluteTurretRotations() * Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
//         turretMotor.getConfigurator().setPosition(motorRotations);
//     }

//     public boolean atTarget() {
//         return Math.abs(getTurretRotations() - targetPositionRotations) < (Units.radiansToDegrees(Settings.Turret.Constants.toleranceRadians) / 360.0);
//     }

//     public void setTarget(double rot) {
//         targetPositionRotations = MathUtil.clamp(rot, Settings.Turret.Constants.TURRET_MIN_ROTATIONS, Settings.Turret.Constants.TURRET_MAX_ROTATIONS);
//     }

//     public double getTurretRotations() {
//         return turretMotor.getPosition().getValueAsDouble() / Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
//     }

//     public double getMotorRotations() {
//         return turretMotor.getPosition().getValueAsDouble();
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putBoolean("Turret/atTarget", atTarget());
//         SmartDashboard.putNumber("Turret/absPositionCRT", getAbsoluteTurretRotations());
//         SmartDashboard.putNumber("Turret/motorPosition", getMotorRotations());
//         SmartDashboard.putNumber("Turret/turretPosition", getTurretRotations());
//         SmartDashboard.putNumber("Turret/targetTurretPosition", targetPositionRotations);

//         if (!isSeeded && Math.abs(turretMotor.getRotorVelocity().getValueAsDouble()) < 1) {
//             setCurrentPosition();
//             isSeeded = true;
//         }

//         if (isSeeded) {
//             double targetMotorRots = targetPositionRotations * Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
//             turretMotor.setControl(targetPosition.withPosition(targetMotorRots));
//         }
//     }   

//     public SysIdRoutine getSysIdRoutine() {
//         return SysId.getRoutine(
//                 2,
//                 6,
//                 "Turret",
//                 voltage -> setVoltageOverride(Optional.of(voltage)),
//                 () -> this.turretMotor.getPosition().getValueAsDouble(),
//                 () -> this.turretMotor.getVelocity().getValueAsDouble(),
//                 () -> this.turretMotor.getMotorVoltage().getValueAsDouble(),
//                 this);
//     }

//     private void setVoltageOverride(Optional<Double> volts) {
//     }
// }
package frc.robot.subsystems.Turret;

import java.util.Optional;

import frc.robot.constants.Motors.TurretConstants;
import frc.robot.constants.Settings;
import frc.robot.util.SysId;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Turret extends SubsystemBase {

    private TalonFX turretMotor;
    private CANcoder turretEncoder; // 60T absolute encoder

    private PositionVoltage targetPosition;
    private double targetPositionRotations;

    private boolean isSeeded = false;

    // --- RATIOS ---
    // 60T gear → turret
    private static final double GEAR_TO_TURRET_RATIO = 136.0 / 14.0; // ≈ 9.714

    // motor → turret (already defined in constants)
    private static final double MOTOR_TO_TURRET = Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;

    public Turret() {
        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR, Settings.upper);

        turretEncoder = new CANcoder(TurretConstants.TURRET_ENCODER_TURRET, Settings.upper);

        targetPosition = new PositionVoltage(0).withEnableFOC(true);

        turretMotor.getConfigurator().apply(TurretConstants.turretConfigs.getConfiguration());
        turretEncoder.getConfigurator().apply(TurretConstants.turretEncoderTurret.getConfiguration());
    }

    private double lastAbsGearRotations = 0;
    private int gearWrapCount = 0;
    private double currentTurretRotations = 0;
    private boolean wrapTrackingInitialized = false;

    private void updatePosition() {
        double absGearRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();

        if (!wrapTrackingInitialized) {
            lastAbsGearRotations = absGearRotations;
            gearWrapCount = 0;
            currentTurretRotations = absGearRotations / GEAR_TO_TURRET_RATIO;
            wrapTrackingInitialized = true;
            return;
        }

        double delta = absGearRotations - lastAbsGearRotations;
        if (delta < -0.5) {
            gearWrapCount++;
        } else if (delta > 0.5) {
            gearWrapCount--;
        }

        lastAbsGearRotations = absGearRotations;
        currentTurretRotations = (gearWrapCount + absGearRotations) / GEAR_TO_TURRET_RATIO;
    }

    // --- ABSOLUTE MULTI-TURN POSITION ---
    public double getAbsoluteTurretRotations() {
        return currentTurretRotations;
    }

    public Rotation2d getAbsoluteTurretRotation2d() {
        return Rotation2d.fromRotations(getAbsoluteTurretRotations());
    }

    // --- SEED MOTOR ENCODER FROM ABSOLUTE ---
    public void setCurrentPosition() {
        double absGearRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
        lastAbsGearRotations = absGearRotations;
        gearWrapCount = 0;
        wrapTrackingInitialized = false;

        double turretRotations = absGearRotations/GEAR_TO_TURRET_RATIO;
        turretMotor.setPosition(turretRotations * MOTOR_TO_TURRET);
    }

    public void reset() {
        turretMotor.setPosition(0);
        turretEncoder.setPosition(0);
    }

    // --- CONTROL ---
    public boolean atTarget() {
        return Math.abs(getAbsoluteTurretRotations()
                - targetPositionRotations) < (Units.radiansToDegrees(Settings.Turret.Constants.toleranceRotations)
                        / 360.0);
    }

    public void setTarget(double rot) {
        targetPositionRotations = MathUtil.clamp(
                rot,
                Settings.Turret.Constants.TURRET_MIN_ROTATIONS,
                Settings.Turret.Constants.TURRET_MAX_ROTATIONS);
    }

    public double getTurretRotations() {
        return turretMotor.getPosition().getValueAsDouble() / MOTOR_TO_TURRET;
    }

    public double getMotorRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        updatePosition();

        // Seed once when stationary
        if (!isSeeded && Math.abs(turretMotor.getRotorVelocity().getValueAsDouble()) < 1) {
            turretMotor.setPosition(currentTurretRotations * MOTOR_TO_TURRET);
            isSeeded = true;
        }

        // Dashboard
        SmartDashboard.putBoolean("Turret/atTarget", atTarget());
        SmartDashboard.putNumber("Turret/absPosition", getAbsoluteTurretRotations());
        SmartDashboard.putNumber("Turret/absRaw", turretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/motorPosition", getMotorRotations());
        SmartDashboard.putNumber("Turret/currentTurretRotations", currentTurretRotations);
        SmartDashboard.putNumber("Turret/motorSeedValue", currentTurretRotations * MOTOR_TO_TURRET);
        SmartDashboard.putNumber("Turret/turretPosition", getTurretRotations());
        SmartDashboard.putNumber("Turret/targetTurretPosition", targetPositionRotations);
        SmartDashboard.putNumber("Turret/turretEncoder", turretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putBoolean("Turret/isSeeded", isSeeded);

        // Closed-loop control
        if (isSeeded) {
            double targetMotorRots = targetPositionRotations * MOTOR_TO_TURRET;
            turretMotor.setControl(targetPosition.withPosition(targetMotorRots));
        }
    }

    // --- SYSID ---
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