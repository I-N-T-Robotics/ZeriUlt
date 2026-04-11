package frc.robot.subsystems.Intake;

import frc.robot.constants.Motors.IntakeConstants;
import frc.robot.constants.Settings;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private TalonFX intakePivot;
    private TalonFX intakeMotor1;

    private boolean isIntaking;
    private double targetPos;

    private final Debouncer debouncer = new Debouncer(1, DebounceType.kRising);

    private final DutyCycleOut power = new DutyCycleOut(0).withEnableFOC(true);
    private final PositionVoltage pos = new PositionVoltage(0).withEnableFOC(true);

    public Intake() {
        intakePivot = new TalonFX(IntakeConstants.PIVOT, Settings.upper);
        intakePivot.setNeutralMode(NeutralModeValue.Brake);
    


        intakeMotor1 = new TalonFX(IntakeConstants.DRIVE, Settings.upper);
        intakeMotor1.setNeutralMode(NeutralModeValue.Coast);

        isIntaking = false;
    }

    public boolean isStalling() {
        boolean stall = intakePivot.getSupplyCurrent().getValueAsDouble() > 40;
        return debouncer.calculate(stall);
    }

    public void deploy() {
            intakePivot.setControl(
                power.withOutput(0.1)
            );
            // positionVoltage
            // .withPosition(Settings.Intake.DEPLOYED_POSITION));
    }

    public void ControlledDeploy() {
        intakePivot.setControl(
               pos.withPosition(0.23)
            );
    }

    // public void undeploy() {
    //     intakePivot.setControl(
    //         positionVoltage
    //         .withPosition(Settings.Intake.UP_POSITION));
    // }

    public void toggleIntake() {
        isIntaking = !isIntaking;

        intakeMotor1.setControl(
            power.withOutput(isIntaking ? 1 : 0)
        );
    }

    public void intake() {
        intakeMotor1.setControl(
           power.withOutput(1)
        );
    }

    public void outtake() {
        intakeMotor1.setControl(
           power.withOutput(-1)
        );
    }

    public void stopIntake() {
        intakeMotor1.stopMotor();
    }

    public void stopIntakePivot() {
        intakePivot.stopMotor();
    }

    public void setIntake(double p) {
        targetPos = p;
    }


    public double getIntakePosition() {
        return intakePivot.getPosition().getValueAsDouble();
    }

    public double getIntakeSpeed() {
        return intakeMotor1.getVelocity().getValueAsDouble();
    }

    public boolean intakeAtDeployPosition() {
        return Math.abs(getIntakePosition() - Settings.Intake.DEPLOYED_POSITION) < Settings.Intake.INTAKE_POSITION_TOLERANCE;
    }

    public boolean intakeAtUndeployPosition() {
        return Math.abs(getIntakePosition() - Settings.Intake.UP_POSITION) < Settings.Intake.INTAKE_POSITION_TOLERANCE;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/position", getIntakePosition());
        SmartDashboard.putNumber("Intake/Speed", getIntakeSpeed());
        SmartDashboard.putNumber("Intake/Target", targetPos);
        SmartDashboard.putBoolean("Intake/atDeployedPosition", intakeAtDeployPosition());
        SmartDashboard.putBoolean("Intake/atUndeployedPosition", intakeAtUndeployPosition());

        // intakePivot.setControl(pos.withPosition(targetPos));
    }
}
