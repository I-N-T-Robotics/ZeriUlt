

package frc.robot.subsystems.Turret;
 
import java.util.Optional;
 
import frc.robot.constants.Motors.TurretConstants;
import frc.robot.constants.Settings;
import frc.robot.util.SysId;
 
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private CANcoder turretEncoder;
 
    private PositionVoltage targetPosition;
    private double targetPositionRotations;
 
    // With FusedCANcoder + SensorToMechanismRatio set in constants,
    // turretMotor.getPosition() returns turret rotations directly.
    // No manual seeding, no wrap counting, no MOTOR_TO_TURRET conversions needed here.
 
    public Turret() {
        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR, Settings.upper);
        turretEncoder = new CANcoder(TurretConstants.TURRET_ENCODER_TURRET, Settings.upper);
 
        targetPosition = new PositionVoltage(0).withEnableFOC(true);
 
        turretMotor.getConfigurator().apply(TurretConstants.turretConfigs.getConfiguration());
        turretEncoder.getConfigurator().apply(TurretConstants.turretEncoderTurret.getConfiguration());
 
        // Hold current position on startup instead of driving to 0
        targetPositionRotations = getAbsoluteTurretRotations();
    }
 
    // Returns turret rotations directly — FusedCANcoder handles multi-turn tracking
    // internally at high frequency, so this is accurate even through hardstops and
    // fast movement. Position is retained across deploys since CANcoder is absolute.
    public double getAbsoluteTurretRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }
 
    public Rotation2d getAbsoluteTurretRotation2d() {
        return Rotation2d.fromRotations(getAbsoluteTurretRotations());
    }
 
    // --- CONTROL ---
    public boolean atTarget() {
        return Math.abs(getAbsoluteTurretRotations() - targetPositionRotations)
                < (Units.radiansToDegrees(Settings.Turret.Constants.toleranceRotations) / 360.0);
    }

    public double getTargetPositionRotations() {
        return targetPositionRotations;
    }
 
    public void setTarget(double rot) {
        targetPositionRotations = MathUtil.clamp(
                rot,
                Settings.Turret.Constants.TURRET_MIN_ROTATIONS,
                Settings.Turret.Constants.TURRET_MAX_ROTATIONS);
    }
 
    // turretMotor.getPosition() already returns turret rotations via FusedCANcoder
    public double getTurretRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }
 
    // Raw motor rotations (before SensorToMechanismRatio scaling) for diagnostics
    public double getMotorRotations() {
        return turretMotor.getPosition().getValueAsDouble()
                * Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH;
    }

    public boolean isAtHardstop() {
        return turretMotor.getSupplyCurrent().getValueAsDouble() > Settings.Turret.Constants.STALL_CURRENT_THRESHOLD;
    }

    public void driveToHardstop() {
        turretMotor.setControl(new VoltageOut(Settings.Turret.Constants.ZEROING_VOLTAGE).withEnableFOC(true));
    }

    public void resetToZero() {
        turretEncoder.setPosition(0);
        targetPositionRotations = 0;
    }
 
    @Override
    public void periodic() {
        // Dashboard
        SmartDashboard.putBoolean("Turret/atTarget", atTarget());
        SmartDashboard.putNumber("Turret/absPosition", getAbsoluteTurretRotations());
        SmartDashboard.putNumber("Turret/absRaw", turretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/turretPosition", getTurretRotations());
        SmartDashboard.putNumber("Turret/targetTurretPosition", targetPositionRotations);
 
        // targetPositionRotations is in turret rotations.
        // FusedCANcoder + SensorToMechanismRatio means the TalonFX PID
        // also operates in turret rotations — no conversion needed.
        turretMotor.setControl(targetPosition.withPosition(targetPositionRotations));
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
