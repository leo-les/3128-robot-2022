package frc.team3128.subsystems; 

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.infrastructure.NAR_EMotor;
import frc.team3128.Constants;
import frc.team3128.common.hardware.PicoColorSensor;
import frc.team3128.common.hardware.PicoColorSensor.RawColor;
import edu.wpi.first.wpilibj.DriverStation; 
import static frc.team3128.Constants.VisionConstants.*;

import static frc.team3128.Constants.HopperConstants.*;

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private NAR_TalonSRX m_hopper1, m_hopper2;
    //private DoubleSolenoid m_hopperSolenoid;
    private Encoder m_encoder;

    private PicoColorSensor m_colorSensor;

    public Hopper() {
        configMotors();
        configEncoders();
        m_colorSensor = new PicoColorSensor();
        // resetEncoder();
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private void configMotors() {
        m_hopper1 = new NAR_TalonSRX(HOPPER_MOTOR_ID);
        m_hopper2 = new NAR_TalonSRX(HOPPER_MOTOR_2_ID);

        // set CAN status frame periods
        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);

        m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_hopper2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
    }

    private void configEncoders() {
        m_encoder = new Encoder(HOPPER_DIO_PIN1, HOPPER_DIO_PIN2);
        // m_encoder.setDistancePerPulse(2.5*Math.PI);
        m_encoder.setReverseDirection(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Enc", m_encoder.getDistance());
    }

    /**
     * @return true if hopper has reversed to desired distance, false if retracted
     */
    public boolean isReversed() {
        return m_encoder.getDistance() <= HOPPER_MAX_REVERSE_DISTANCE;
    }

    /**
     * Runs the hopper using the HOPPER_MOTOR_POWER constant
     */
    public void runHopper() {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        m_hopper2.set(HOPPER_MOTOR_2_POWER);
    }

    public void runHopper(double power) {
        m_hopper1.set(power);
        m_hopper2.set(power + 0.1);
    }

    public void reverseHopper(double power) {
        m_encoder.reset();
        m_hopper1.set(power);
    }

    /**
     * Stops the hopper - sets power to 0
     */
    public void stopHopper() {
        m_hopper1.set(0);
        m_hopper2.set(0);
    }

    public void resetMotorEncoder(){
        m_hopper1.setSelectedSensorPosition(0);
    }

    public void setNeutralMode(NeutralMode mode) {
        m_hopper1.setNeutralMode(mode);
        m_hopper2.setNeutralMode(mode);
    }

    public boolean getBallBottomLocation() {
        return m_colorSensor.getProximity0() > PROXIMITY_SENSOR_THRESHOLD;
    }

    public boolean getBallTopLocation() {
        return m_colorSensor.getProximity1() > PROXIMITY_SENSOR_THRESHOLD;
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