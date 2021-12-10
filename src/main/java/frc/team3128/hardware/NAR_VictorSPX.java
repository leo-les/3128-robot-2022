package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class NAR_VictorSPX extends NAR_Motor<WPI_VictorSPX> {


    //Lazy Logic Variables
    protected double prevValue = 0;
	protected ControlMode prevControlMode = ControlMode.Disabled;

    NAR_VictorSPX(int deviceNumber) {
		super(deviceNumber);
	}

	@Override
	public void constructReal() {
		this.motorController = new WPI_VictorSPX(deviceNumber);
		motorController.enableVoltageCompensation(true);
		motorController.configVoltageCompSaturation(12, 10);
	}

    @Override
	public void set(ControlMode controlMode, double value) {
		if (value != prevValue || controlMode != prevControlMode) {
			motorController.set(controlMode, value);
			prevValue = value;
		}
	}

	@Override
	public void set(double value) {
		motorController.set(prevControlMode, value);
	}

    @Override
	public double getSetpoint() {
		return prevValue;
	}
}