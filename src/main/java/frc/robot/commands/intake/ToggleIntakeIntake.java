package frc.robot.commands.intake;

import frc.robot.subsystems.Intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntakeIntake extends Command {
    private final Intake intake;

    public ToggleIntakeIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.toggleIntake();
    }
}