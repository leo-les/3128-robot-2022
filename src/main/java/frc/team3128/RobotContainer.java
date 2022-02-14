package frc.team3128;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team3128.commands.ArcadeDrive;
import frc.team3128.commands.CmdBallJoystickPursuit;
import frc.team3128.commands.CmdBallPursuit;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.Limelight;

import frc.team3128.subsystems.NAR_Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private NAR_Drivetrain m_drive;
<<<<<<< Updated upstream
=======
    private Shooter m_shooter;
    // private Intake m_intake;   
    private Hopper m_hopper;
    // private Climber m_climber;
>>>>>>> Stashed changes

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private Limelight ballLimelight;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    private String trajJson = "paths/jude_path_o_doom.wpilib.json";
    private Trajectory trajectory = new Trajectory();
    private Command auto;

    private boolean DEBUG = false;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();
<<<<<<< Updated upstream
=======
        m_shooter = Shooter.getInstance();
        // m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        // m_climber = Climber.getInstance();
>>>>>>> Stashed changes

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

<<<<<<< Updated upstream
        ballLimelight = new Limelight("limelight-sog", Constants.VisionContants.BALL_LL_ANGLE, Constants.VisionContants.BALL_LL_HEIGHT, 0, 0);
=======
        m_shooterLimelight = new Limelight("limelight-pog", VisionConstants.TOP_CAMERA_ANGLE, 
                                                            VisionConstants.TOP_CAMERA_HEIGHT, 
                                                            VisionConstants.TOP_FRONT_DIST, 0); 
        m_balLimelight = new Limelight("limelight-sog", VisionConstants.BALL_LL_ANGLE, 
                                                        VisionConstants.BALL_LL_HEIGHT, 
                                                        VisionConstants.BALL_LL_FRONT_DIST, 0);
>>>>>>> Stashed changes

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));

        try {
            Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajJson);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException ex) {
            DriverStation.reportError("Me me no open trajectory: " + trajJson, ex.getStackTrace());
        }


        initAutos();
        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {
<<<<<<< Updated upstream
        m_rightStick.getButton(1).whenHeld(new CmdBallJoystickPursuit(m_drive, ballLimelight, m_rightStick));
        m_rightStick.getButton(3).whenPressed(new CmdBallPursuit(m_drive, ballLimelight));
=======
        // Buttons...
        // right:
        // 1 (trigger): intake 
        // 2: shoot
        // 8: climb
        // 9: stop climb
        //
        // left:
        // 9: make climber go up
        // 10: make climb go down
        // 11: extend climber piston
        // 12: retract climber piston
        // 15: push climber all the way to top magnet
        // 14: push climber all the way to bottom magnet

        //RIGHT
        // m_rightStick.getButton(1).whenHeld(extendIntakeAndRun);
                                // .whenReleased(retractHopperCommand); Garrison said no to this
        
        m_rightStick.getButton(2).whenPressed(shootCommand)
                                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot,m_shooter), new InstantCommand(m_shooterLimelight::turnLEDOff)));

        m_rightStick.getButton(3).whenPressed(retractHopperCommand);

        // // // // m_rightStick.getButton(5).whenPressed(new SequentialCommandGroup(new InstantCommand(m_intake::ejectIntake, m_intake), new RunCommand(m_intake::runIntakeBack, m_intake)).withTimeout(0.15))
                                // // .whenReleased(new InstantCommand(m_intake::stopIntake, m_intake));

        // // m_rightStick.getButton(6).whenPressed(m_intake::retractIntake, m_intake);

        m_rightStick.getButton(11).whenPressed(manualShoot) //manualShoot
                                .whenReleased(new ParallelCommandGroup(new InstantCommand(m_shooter::stopShoot,m_shooter), new InstantCommand(m_shooterLimelight::turnLEDOff)));

        //LEFT
        m_leftStick.getButton(1).whenHeld(lowerHubShoot);

        // // m_leftStick.getButton(9).whenPressed(new InstantCommand(m_climber::extendBoth, m_climber))
                                // // .whenReleased(new InstantCommand(m_climber::stopBoth, m_climber));

        // // m_leftStick.getButton(10).whenPressed(new InstantCommand(m_climber::retractBoth, m_climber))
                                // // .whenReleased(new InstantCommand(m_climber::stopBoth, m_climber));

        // // m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::extendPiston, m_climber));
        // // m_leftStick.getButton(12).whenPressed(new InstantCommand(m_climber::retractPiston, m_climber));
        // // m_leftStick.getButton(16).whenPressed(new InstantCommand(m_climber::engageBreak, m_climber));
        // // m_leftStick.getButton(15).whenPressed(new InstantCommand(m_climber::disengageBreak, m_climber));

        


        //climber buttons (uncomment when testing climber)
        // m_leftStick.getButton(10).whenPressed(new CmdClimbEncoder(m_climber, ClimberConstants.SMALL_VERTICAL_DISTANCE));
        // m_leftStick.getButton(11).whenPressed(new InstantCommand(m_climber::extendArm, m_climber));
        // m_leftStick.getButton(12).whenPressed(new InstantCommand(m_climber::retractArm, m_climber));
        // m_leftStick.getButton(15).whenPressed(new CmdClimbExtend(m_climber));
        // m_leftStick.getButton(14).whenPressed(new CmdClimbRetract(m_climber));

        // //m_rightStick.getButton(8).whenPressed(climbCommand);
        // m_rightStick.getButton(8).whenReleased(new InstantCommand(m_climber::climberStop, m_climber));
        // m_rightStick.getButton(9).whenPressed(new InstantCommand(m_climber::climberStop, m_climber));
>>>>>>> Stashed changes
    }

    private void initAutos() {
<<<<<<< Updated upstream
        auto = new RamseteCommand(trajectory, 
                                m_drive::getPose,
                                new RamseteController(Constants.DriveConstants.RAMSETE_B, Constants.DriveConstants.RAMSETE_ZETA),
                                new SimpleMotorFeedforward(Constants.DriveConstants.kS,
                                                            Constants.DriveConstants.kV,
                                                            Constants.DriveConstants.kA),
                                Constants.DriveConstants.DRIVE_KINEMATICS,
                                m_drive::getWheelSpeeds,
                                new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                                new PIDController(Constants.DriveConstants.RAMSETE_KP, 0, 0),
                                m_drive::tankDriveVolts,
                                m_drive)
                                .andThen(() -> m_drive.stop(), m_drive);
=======

        try {
            for (int i = 0; i < trajJson.length; i++) {
                // Get a path from the string specified in trajJson, and load it into trajectory[i]
                Path path = Filesystem.getDeployDirectory().toPath().resolve(trajJson[i]);
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(path);
            }
        } catch (IOException ex) {
            DriverStation.reportError("IOException opening trajectory:", ex.getStackTrace());
        }

        initialPoses = new HashMap<Command, Pose2d>();

        retractHopperCommand = new CmdRetractHopper(m_hopper);
        // climbCommand = new CmdClimb(m_climber);
        
        // intakeCargoCommand = new CmdIntakeCargo(m_intake, m_hopper);

        // extendIntakeAndRun = new SequentialCommandGroup(new CmdExtendIntake(m_intake).withTimeout(0.1), intakeCargoCommand);

        //this shoot command is the ideal one with all capabilities
        shootCommand = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn),
                        new CmdRetractHopper(m_hopper), 
                        new ParallelCommandGroup(
                            new CmdAlign(m_drive, m_shooterLimelight), 
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootRPM(m_shooter, m_shooter.calculateMotorVelocityFromDist(
                                            m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT) - 3)))
                                            // this is -3 because magic number (ll math on kitbot overestimates by -3 for some reason)
        );

        //use this shoot command for testing
        manualShoot = new SequentialCommandGroup(
                        new InstantCommand(m_shooterLimelight::turnLEDOn), 
                        new CmdRetractHopper(m_hopper),
                        new ParallelCommandGroup(
                            new CmdHopperShooting(m_hopper, m_shooter::isReady),
                            new CmdShootRPM(m_shooter, m_shooter.calculateMotorVelocityFromDist(
                                            m_shooterLimelight.calculateDistToTopTarget(Constants.VisionConstants.TARGET_HEIGHT) - 3)))
                                            // this is -3 because magic number (ll math on kitbot overestimates by -3 for some reason)
        );

        lowerHubShoot = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper),
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter,1250))
        );
                        
        shootCommand2 = new SequentialCommandGroup(
                            new CmdRetractHopper(m_hopper),  
                            new ParallelCommandGroup(
                                new CmdHopperShooting(m_hopper, m_shooter::isReady),
                                new CmdShootRPM(m_shooter,3000))
        );


        //AUTONOMOUS ROUTINES
        // auto_2BallBot = new SequentialCommandGroup(

        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),

        //                     new ParallelDeadlineGroup(
        //                         trajectoryCmd(0).andThen(m_drive::stop, m_drive),

        //                         new InstantCommand(() -> {
        //                         // m_intake.runIntake();
        //                         m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     ).withTimeout(4)

        // );
        
        // auto_2BallMid = new SequentialCommandGroup(

        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),
                            
        //                     new ParallelDeadlineGroup(
        //                         trajectoryCmd(1).andThen(m_drive::stop, m_drive),

        //                         new InstantCommand(() -> {
        //                             // m_intake.runIntake();
        //                             m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3500)
        //                     ).withTimeout(4)

        // );

        // auto_2BallTop = new SequentialCommandGroup(

        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),
                                            
        //                     new ParallelDeadlineGroup(
        //                         trajectoryCmd(2).andThen(m_drive::stop, m_drive),

        //                         new InstantCommand(() -> {
        //                             // m_intake.runIntake();
        //                             m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     ).withTimeout(4)

        // );

        // auto_3BallHook = new SequentialCommandGroup(

        //     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //     new ParallelCommandGroup(
        //         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //         new CmdShootRPM(m_shooter, 3350)
        //     ).withTimeout(3),

        //     // new CmdExtendIntake(m_intake).withTimeout(0.1),
        //     new ParallelDeadlineGroup(
        //         new SequentialCommandGroup(
        //             trajectoryCmd(3),
        //             trajectoryCmd(4),
        //             new InstantCommand(m_drive::stop, m_drive)
        //         ),
        //         new InstantCommand(() -> {
        //             // m_intake.runIntake();
        //             m_hopper.runHopper();
        //         // }, m_intake, m_hopper)
        //     ),

        //     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //     new ParallelCommandGroup(
        //         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //         new CmdShootRPM(m_shooter, 3250)
        //     ).withTimeout(3)

        // );
        
        // auto_3BallTerminal = new SequentialCommandGroup(

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     ).withTimeout(4), // Edit this timeout when tested

        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),
        //                     new ParallelDeadlineGroup(
        //                         new SequentialCommandGroup(
        //                             trajectoryCmd(5),
        //                             trajectoryCmd(6),
        //                             trajectoryCmd(7),
        //                             trajectoryCmd(8),
        //                             new InstantCommand(m_drive::stop, m_drive)
        //                         ),
        //                         new InstantCommand(() -> {
        //                             // m_intake.runIntake();
        //                             m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     ).withTimeout(4)

        // );

        // auto_3BallHersheyKiss = new SequentialCommandGroup(
        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     ).withTimeout(4),
                            
        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),
        //                     new ParallelDeadlineGroup(
        //                         new SequentialCommandGroup(
        //                             trajectoryCmd(9),
        //                             trajectoryCmd(10),
        //                             new InstantCommand(m_drive::stop, m_drive)
        //                         ),
        //                         new InstantCommand(() -> {
        //                             // m_intake.runIntake();
        //                             m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     )
        // );

        // auto_4BallE = new SequentialCommandGroup(

        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),
        //                     new ParallelDeadlineGroup(
        //                         new SequentialCommandGroup(
        //                             trajectoryCmd(11),
        //                             new InstantCommand(m_drive::stop, m_drive)
        //                         ),
        //                         new InstantCommand(() -> {
        //                             // m_intake.runIntake();
        //                             m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     ),

        //                     // new CmdExtendIntake(m_intake).withTimeout(0.1),
        //                     new ParallelDeadlineGroup(
        //                         new SequentialCommandGroup(
        //                             trajectoryCmd(12),
        //                             new InstantCommand(m_drive::stop, m_drive)
        //                         ),
        //                         new InstantCommand(() -> {
        //                             // m_intake.runIntake();
        //                             m_hopper.runHopper();
        //                         // }, m_intake, m_hopper)
        //                     ),

        //                     new CmdRetractHopper(m_hopper).withTimeout(0.5),
        //                     new ParallelCommandGroup(
        //                         new CmdHopperShooting(m_hopper, m_shooter::isReady),
        //                         new CmdShootRPM(m_shooter, 3000)
        //                     )

        // );

        // Setup auto-selector
        NarwhalDashboard.addAuto("2 Ball Bottom", auto_2BallBot);
        NarwhalDashboard.addAuto("2 Ball Mid", auto_2BallMid);
        NarwhalDashboard.addAuto("2 Ball Top", auto_2BallTop);
        NarwhalDashboard.addAuto("3 Ball Hook", auto_3BallHook);
        NarwhalDashboard.addAuto("3 Ball Terminal", auto_3BallTerminal);
        NarwhalDashboard.addAuto("3 Ball Hershey Kiss", auto_3BallHersheyKiss);
        NarwhalDashboard.addAuto("4 Ball E", auto_4BallE);
>>>>>>> Stashed changes
    }

    private void dashboardInit() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            SmartDashboard.putData("Drivetrain", m_drive);
<<<<<<< Updated upstream
=======
            // SmartDashboard.putData("Intake", m_intake);
            SmartDashboard.putData("Hopper", m_hopper);
            SmartDashboard.putData("Shooter", m_shooter);
            SmartDashboard.putBoolean("Shooter at Setpoint", m_shooter.isReady());
            SmartDashboard.putString("Shooter state", m_shooter.getState().toString());
            SmartDashboard.putNumber("Shooter Setpoint", m_shooter.getSetpoint());
            SmartDashboard.putNumber("Shooter RPM", m_shooter.getMeasurement());
>>>>>>> Stashed changes
        }
            
    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        m_drive.resetPose(trajectory.getInitialPose()); // change this if the trajectory being run changes
        return auto;
    }
}
