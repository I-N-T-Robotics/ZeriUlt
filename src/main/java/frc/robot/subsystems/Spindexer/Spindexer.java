package frc.robot.subsystems.Spindexer;

import frc.robot.constants.Motors;
import frc.robot.constants.Settings;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

    private TalonFX intakeSpindexMotor;
    private TalonFX farSpindexMotor;
    private TalonFX transitionMotor;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

    public Spindexer() {
        intakeSpindexMotor = new TalonFX(Motors.SpindexerConstants.INTAKE_SPINDEXER_MOTOR, Settings.upper);
        intakeSpindexMotor.setNeutralMode(NeutralModeValue.Coast);

        farSpindexMotor = new TalonFX(Motors.SpindexerConstants.FAR_SPINDEXER_MOTOR, Settings.upper);
        farSpindexMotor.setNeutralMode(NeutralModeValue.Coast);

        transitionMotor = new TalonFX(Motors.SpindexerConstants.TRANSITION_MOTOR, Settings.upper);
        transitionMotor.setNeutralMode(NeutralModeValue.Coast);

        intakeSpindexMotor.setControl(new Follower(
            Motors.SpindexerConstants.FAR_SPINDEXER_MOTOR, MotorAlignmentValue.Aligned));

        intakeSpindexMotor.getConfigurator().apply(Motors.SpindexerConstants.intakeSpindexerMotorConfig.getConfiguration());
        farSpindexMotor.getConfigurator().apply(Motors.SpindexerConstants.farSpindexerMotorConfig.getConfiguration());
        transitionMotor.getConfigurator().apply(Motors.SpindexerConstants.transitionMotorConfig.getConfiguration());
    }

    public boolean getIsStalling() {
        return (transitionMotor.getSupplyCurrent().getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT) ||
               (farSpindexMotor.getSupplyCurrent().getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT);
    }

    public void startTransition() {
        transitionMotor.setControl(velocityVoltage
            .withVelocity(Settings.Spindexer.SPINDEXER_RPS * Settings.Spindexer.TRANSITION_TO_SPEED_RATIO));
    }

    public void stopTransition() {
        transitionMotor.setControl(velocityVoltage.withVelocity(0));
    }

    public void reverseTransition() {
        transitionMotor.setControl(velocityVoltage
            .withVelocity(-Settings.Spindexer.SPINDEXER_RPS * Settings.Spindexer.TRANSITION_TO_SPEED_RATIO));
    }

    // TalonFX returns RPS natively
    public double getTransitionRPS() {
        return transitionMotor.getVelocity().getValueAsDouble();
    }

    // TRANSITION_MIN_SPEED and tolerance should both be in RPS in Settings
    public boolean transitionAtSpeed() {
        return Math.abs(getTransitionRPS() - Settings.Spindexer.TRANSITION_MIN_RPS)
                < Settings.Spindexer.TRANSITION_RPS_TOLERANCE;
    }

    public void startSpindexer() {
        farSpindexMotor.setControl(velocityVoltage
            .withVelocity(Settings.Spindexer.SPINDEXER_RPS));
    }

    public void stopSpindexer() {
        farSpindexMotor.setControl(velocityVoltage.withVelocity(0));
    }

    public void reverseSpindexer() {
        farSpindexMotor.setControl(velocityVoltage
            .withVelocity(-Settings.Spindexer.SPINDEXER_RPS));
    }

    public double getSpindexerRPS() {
        return farSpindexMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer/SpindexerRPS", getSpindexerRPS());
        SmartDashboard.putNumber("Spindexer/TransitionRPS", getTransitionRPS());
        SmartDashboard.putBoolean("Spindexer/IsStalling", getIsStalling());
    }
}