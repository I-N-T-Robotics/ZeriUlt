package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Gains;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class AimAndDrive extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final Supplier<Translation2d> targetTranslationSupplier;

    private final PIDController omegaController;
    private final SwerveRequest.FieldCentric driveRequest;

    public AimAndDrive(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            Supplier<Translation2d> targetTranslationSupplier) {

        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.targetTranslationSupplier = targetTranslationSupplier;

        this.omegaController = new PIDController(
                2,
                0,
                0);
        
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.driveRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        Translation2d targetTranslation = targetTranslationSupplier.get();

        Translation2d robotToTarget = targetTranslation.minus(currentPose.getTranslation());
        Rotation2d targetRotation = robotToTarget.getAngle();

        double omegaRadPerSec = omegaController.calculate(
                currentPose.getRotation().getRadians(),
                targetRotation.getRadians()
        );
double deadband = 0.1;
        double xVelocity = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), deadband);
        double yVelocity = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), deadband);

       
        drivetrain.setControl(
             driveRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(omegaRadPerSec)
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
