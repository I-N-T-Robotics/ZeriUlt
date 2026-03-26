package frc.robot.commands.Spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.Turret.Turret;

public class SpindexerStart extends Command {
    private final Spindexer spindexer;
    private final Turret turret;

    public SpindexerStart(Spindexer spindexer, Turret turret) {
        this.spindexer = spindexer;
        this.turret = turret;
        addRequirements(spindexer, turret);
    }

    @Override
    public void execute() {
        // if (spindexer.transitionAtSpeed() && turret.atTarget()) {
        //     spindexer.startSpindexer();
        // }
        if (spindexer.getIsStalling()) {
            spindexer.reverseSpindexer();
            spindexer.reverseTransition();
        } else {
            spindexer.startTransition();
            spindexer.startSpindexer();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpindexer();
        spindexer.stopTransition();
    }
}