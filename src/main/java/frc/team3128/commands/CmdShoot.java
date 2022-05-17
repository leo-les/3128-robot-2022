package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdAlign;
import frc.team3128.subsystems.Shooter.ShooterState;

public class CmdShoot extends SequentialCommandGroup {


    public CmdShoot(NAR_Drivetrain m_drive, Hood m_hood, LimelightSubsystem m_ll, Hopper m_hopper, Shooter m_shooter) {
        addCommands(
            
            new InstantCommand(() -> m_ll.turnShooterLEDOn()),
            new CmdRetractHopper(m_hopper).withTimeout(0.5),
            new InstantCommand(() -> m_shooter.setState(ShooterState.UPPERHUB)),
            // new CmdExtendIntake(m_intake),
            new ParallelCommandGroup(
                // new RunCommand(m_intake::runIntake, m_intake),
                new CmdAlign(m_drive, m_ll),
                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                new CmdShootDist(m_shooter, m_hood, m_ll)
                
            )


        );
    }
}
