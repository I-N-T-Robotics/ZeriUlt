package frc.robot.commands.intake;

import frc.robot.subsystems.Intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeIntake extends Command {
    private final Intake intake;

    public IntakeIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
        intake.ControlledDeploy();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}