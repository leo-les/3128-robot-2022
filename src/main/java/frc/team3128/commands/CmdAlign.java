package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdAlign extends CommandBase {

    private enum VisionState {
        SEARCHING, FEEDBACK;
    }

    private NAR_Drivetrain drive;
    private LimelightSubsystem limelights;

    private double goalHorizontalOffset, currHorizontalOffset;
    private double prevError, currError;
    
    private double prevTime, currTime; // seconds
    private int plateauCount, targetFoundCount;
    private boolean isAligned;
    private NetworkTableEntry entry;
    private NetworkTableEntry kf;
    private NetworkTableEntry kp;
    private NetworkTableEntry kd;
    private NetworkTableEntry ki;
    private NetworkTableEntry thresh;

    private VisionState aimState = VisionState.SEARCHING;

    /**
     * Aligns the robot to the hub using the limelight
     * @Requirements Drivetrain  
     */
    public CmdAlign() {
        this.drive = NAR_Drivetrain.getInstance();
        this.limelights = LimelightSubsystem.getInstance();

        goalHorizontalOffset = TX_OFFSET;
        isAligned = false;
        entry = Shuffleboard.getTab("test").add("Error", 0).getEntry();
        kf = Shuffleboard.getTab("test").add("kf",0).getEntry();
        kp = Shuffleboard.getTab("test").add("kp",0).getEntry();
        kd = Shuffleboard.getTab("test").add("kd",0).getEntry();
        ki = Shuffleboard.getTab("test").add("ki",0).getEntry();
        thresh = Shuffleboard.getTab("test").add("Thresh",0).getEntry();
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isAligned = false;
        limelights.turnShooterLEDOn();
        prevTime = RobotController.getFPGATime() / 1e6;
        plateauCount = 0;
    }

    @Override
    public void execute() {
        currTime = RobotController.getFPGATime() / 1e6;
        switch (aimState) {
            case SEARCHING:
                if (limelights.getShooterHasValidTarget())
                    targetFoundCount++;
                else
                    targetFoundCount = 0;
                
                // if target found for enough iterations, switch to FEEDBACK
                if(targetFoundCount > 5) {
                    currHorizontalOffset = limelights.getShooterTX();
                    prevError = goalHorizontalOffset - currHorizontalOffset;
                    aimState = VisionState.FEEDBACK;
                }
                SmartDashboard.putNumber("ll plat count", targetFoundCount);
                break;
            
            case FEEDBACK:
                // if no more valid target, switch to SEARCHING
                if(!limelights.getShooterHasValidTarget()) {
                    aimState = VisionState.SEARCHING;
                    plateauCount = 0;
                    break;
                }

                // turn with PID loop using input as horizontal tx error to target
                currHorizontalOffset = limelights.getShooterTX();
                currError = goalHorizontalOffset - currHorizontalOffset; // currError is positive if we are too far left

                double ff = Math.signum(currError) * kf.getDouble(0);
                double feedbackPower = kp.getDouble(0) * currError + kd.getDouble(0) * (currError - prevError) / (currTime - prevTime) + ff;
                
                
                feedbackPower = MathUtil.clamp(feedbackPower, -1, 1);

                drive.tankDrive(-feedbackPower, feedbackPower);
                SmartDashboard.putNumber("ll feedback power", feedbackPower);
                SmartDashboard.putNumber("ll curr error", currError);
                entry.setNumber(currError);
    
                // if degrees of horizontal tx error below threshold (aligned enough)
                if (Math.abs(currError) < thresh.getDouble(0)) {
                    plateauCount++;
                    if (plateauCount > ALIGN_PLATEAU_COUNT) {
                        isAligned = true;
                    }
                }
                else {
                    isAligned = false;
                    plateauCount = 0;
                }

                prevError = currError;

                break;
                
        }
        prevTime = currTime;
        SmartDashboard.putBoolean("Shooter isAligned", isAligned);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        limelights.turnShooterLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return isAligned;
    }
}