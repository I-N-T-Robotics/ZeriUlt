package frc.robot.commands.intake;

import frc.robot.subsystems.Intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployIntake extends Command {
    private final Intake intake;

    public DeployIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
         intake.deploy();
        //intake.ControlledDeploy();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakePivot();
    }
}