// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Spindexer.SpindexerStart;
import frc.robot.commands.Spindexer.SpindexerStop;
import frc.robot.commands.hood.HoodAim;
import frc.robot.commands.intake.AutoDeploy;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.IntakeIntake;
import frc.robot.commands.intake.IntakeOuttake;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.commands.shooter.ShooterShootTest2;
import frc.robot.commands.shooter.ShooterShootTest3;
import frc.robot.commands.swerve.AimAndDrive;
import frc.robot.commands.turret.AutoAim;
import frc.robot.commands.turret.ResetTurret;
import frc.robot.constants.Field;
import frc.robot.constants.Settings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Vision.LimelightVision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean isFerrying = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final CommandPS5Controller AmanController = new CommandPS5Controller(2);

    //subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Intake intake = new Intake();
    private final Spindexer spindexer = new Spindexer();
    private final LimelightVision limelightVision = new LimelightVision();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        NamedCommands.registerCommand("StartIntake", new IntakeIntake(intake));
        NamedCommands.registerCommand("DeployIntake", new DeployIntake(intake));
        NamedCommands.registerCommand("AutoDeploy", new AutoDeploy(intake));
        NamedCommands.registerCommand("StopIntake", new IntakeStop(intake));
        NamedCommands.registerCommand("StartSpindexer", new SpindexerStart(spindexer, turret, shooter));
        NamedCommands.registerCommand("StopSpindexer", new SpindexerStop(spindexer));
        NamedCommands.registerCommand("StartShooter", new ShooterShoot(shooter, drivetrain, turret, () -> getGoalPosition()));
        NamedCommands.registerCommand("AutoShoot", new AutoShoot(shooter, spindexer, drivetrain));
        NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> shooter.stopShooter(), shooter));

        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoChooser", autonChooser);
        //configureAutons();
        configureDefaultCommands();
        configureButtonBindings();

        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
         //turret.setDefaultCommand(new AutoAim(turret, drivetrain, () -> Field.BLUE_GOAL_CENTER)); //TODO FCHANGE FOR MATCH
        // shooter.setDefaultCommand(new ShooterShoot(shooter, drivetrain, turret, this));
        hood.setDefaultCommand(new HoodAim(hood, drivetrain, this));

        drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX((-AmanController.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                .withVelocityY((-AmanController.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                .withRotationalRate((-AmanController.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        )
        ); 
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {


        // AmanController.L2()
        //     .whileTrue(new DeployIntake(intake));
        AmanController.R1()
            .whileTrue(new DeployIntake(intake));

        AmanController.circle()
            .toggleOnTrue(new IntakeIntake(intake));

        AmanController.L1()
            .toggleOnTrue(new IntakeOuttake(intake, spindexer));

        AmanController.povDown()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //     .toggleOnTrue(new SwerveXMode(drivetrain));

        AmanController.cross()
            .whileTrue(new ParallelCommandGroup(
                new ShooterShoot(shooter, drivetrain, turret, () -> getGoalPosition()), //TODO FCHANGE FOR MATCH
                new SpindexerStart(spindexer, turret, shooter)
            ));

        AmanController.square()
            // .whileTrue(new ResetTurret(turret));
            .toggleOnTrue(new ParallelCommandGroup(
                new ShooterShootTest2(shooter),
                new SpindexerStart(spindexer, turret, shooter)
                //,new HoodAim(hood, drivetrain)
            ));

        AmanController.triangle()
            .toggleOnTrue(new ParallelCommandGroup(
                new ShooterShootTest3(shooter),
                new SpindexerStart(spindexer, turret, shooter)
                //,new HoodAim(hood, drivetrain)
            ));

        AmanController.R3()
            .toggleOnTrue(new AimAndDrive(drivetrain, 
            () -> -AmanController.getLeftY(), 
            () -> -AmanController.getLeftX(), 
            () -> getGoalPosition()));
    }

    public void configureSysids() {
        SysIdRoutine turretSysid = turret.getSysIdRoutine();
        autonChooser.addOption("SysID Turret Dynamic Forward", turretSysid.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Turret Dynamic Backwards", turretSysid.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Turret Quasi Forwards", turretSysid.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Turret Quasi Backwards", turretSysid.quasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public boolean isInTopSide() {
        return drivetrain.getPose().getX() <= Field.WIDTH / 2.0;
    }

    public boolean robotIsOnAllianceSide() {
        Pose2d pose = drivetrain.getPose();
        return Robot.isBlue()
                ? pose.getX() < Units.inchesToMeters(182.11)
                : pose.getX() > Units.inchesToMeters(469.11);
    }

    public Translation2d getGoalPosition() {
        if (robotIsOnAllianceSide()) {
            return Field.hubCenter;
        } else {
            return isInTopSide() ? Field.topFerry : Field.bottomFerry;
        }
    }

    public boolean getIsFerrying() {
        isFerrying = !robotIsOnAllianceSide();
        return isFerrying;
    }

    public Command AddVisionMeasurement() {
        return new frc.robot.commands.vision.AddVisionMeasurement(drivetrain, limelightVision)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .ignoringDisable(true);
    }

  public LimelightVision getLimelightVision() {
        return limelightVision;
    }
}