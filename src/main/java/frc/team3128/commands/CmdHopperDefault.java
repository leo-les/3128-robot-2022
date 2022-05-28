package frc.team3128.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;

public class CmdHopperDefault extends CommandBase {

    private Hopper m_hopper;
    private BooleanSupplier isShooting;

    public CmdHopperDefault(BooleanSupplier isShooting) {
        m_hopper = Hopper.getInstance();
        this.isShooting = isShooting;

        addRequirements(m_hopper);
    }

    @Override
    public void execute() {
        // if shooting, retract gate if ejected and run the hopper
        if (isShooting.getAsBoolean()) { 
            m_hopper.runHopper();
        } else {
            m_hopper.stopHopper();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}