package frc.team3128.subsystems; 

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.team3128.common.hardware.PicoColorSensor;
import frc.team3128.common.hardware.PicoColorSensor.RawColor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import static frc.team3128.Constants.HopperConstants.*;

/**
 * Class for the Hopper Subsystem 
 * (most reliable subsystem eyyyyyyy)
 */

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private PicoColorSensor m_colorSensor;
    private DigitalInput m_bottomDistanceSensor;
    private DigitalInput m_topDistanceSensor;

    private NAR_TalonSRX m_hopper1, m_hopper2;

    private Encoder m_encoder;

    public Hopper() {
        configMotors();
        configEncoders();
        configSensors();
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors() {
        m_hopper1 = new NAR_TalonSRX(HOPPER_MOTOR_ID);
        m_hopper2 = new NAR_TalonSRX(HOPPER_MOTOR_2_ID);

        m_hopper1.setNeutralMode(NeutralMode.Coast);
        m_hopper2.setNeutralMode(NeutralMode.Coast);

        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_hopper1.setControlFramePeriod(ControlFrame.Control_3_General, 20);

        m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_hopper2.setControlFramePeriod(ControlFrame.Control_3_General, 20);
    }

    /**
     * Initializes hopper encoder 
     */
    private void configEncoders() {
        m_encoder = new Encoder(HOPPER_DIO_PIN1, HOPPER_DIO_PIN2);
        m_encoder.setReverseDirection(true);
    }

    private void configSensors() {
        m_colorSensor = new PicoColorSensor();
        m_bottomDistanceSensor = new DigitalInput(0);
        m_topDistanceSensor = new DigitalInput(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Enc", m_encoder.getDistance());
    }

    /**
     * Runs the hopper motors using the default power
     */
    public void runHopper() {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        m_hopper2.set(HOPPER_MOTOR_2_POWER);
    }

    /**
     * Runs the hopper motors using the parameter power
     */
    public void runHopper(double power) {
        m_hopper1.set(power);
        m_hopper2.set(power + 0.1);
    }

    /**
     * Runs hopper motor back at parameter power
     */
    public void reverseHopper(double power) {
        m_hopper1.set(power);
    }

    /**
     * Stops the hopper - sets power to 0
     */
    public void stopHopper() {
        m_hopper1.set(0);
        m_hopper2.set(0);
    }

    /**
     * Resets hopper encoder
     */
    public void resetEncoder(){
        m_encoder.reset();
    }

    /**
     * Gets hopper distance using encoder
     */
    public double getHopperDistance() {
        return m_encoder.getDistance();
    }

    
    public boolean getBallBottomLocation() {
        return m_bottomDistanceSensor.get();
    }

    public boolean getBallTopLocation() {
        return m_topDistanceSensor.get();
    }

    /**
     * Returns true if bottom ball is wrong color, false if ball is right color or missing
     */
    public boolean getWrongBallBottom() {
        RawColor color = m_colorSensor.getRawColor0();
        if (color.red > color.blue*COLOR_SENSOR_TOLERANCE) {
            //return if red
            return DriverStation.getAlliance() != DriverStation.Alliance.Red;
        } else if (color.blue > color.red*COLOR_SENSOR_TOLERANCE) {
            //return if blue
            return DriverStation.getAlliance() != DriverStation.Alliance.Blue;
        } 
        return false;
    }

    /**
     * Returns true if top ball is wrong color, false if ball is right color or missing
     */
    public boolean getWrongBallTop() {
        RawColor color = m_colorSensor.getRawColor1();
        if (color.red > color.blue*1.5) {
            //return if red
            return DriverStation.getAlliance() != DriverStation.Alliance.Red;
        } else if (color.blue > color.red*1.5) {
            //return if blue
            return DriverStation.getAlliance() != DriverStation.Alliance.Blue;
        }
        return false;
    }

}