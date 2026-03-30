package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision.LimelightVision;
import frc.robot.subsystems.Vision.LimelightVision.MegaTagMode;

public class SetMegaTagMode extends InstantCommand {
    
    private MegaTagMode megaTagMode;
    private LimelightVision limelightVision;

    public SetMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    @Override
    public void initialize() {
        super.initialize();
        limelightVision.setMegaTagMode(megaTagMode);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}