package frc.robot.commands.intake;

import frc.robot.subsystems.Intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoDeploy extends Command {
    private final Intake intake;

    public AutoDeploy(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.deploy();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakePivot();
    }
}