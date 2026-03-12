package frc.robot.subsystems.Intaker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.IntakerConstants;

public class IntakerIOPhoenix6 implements IntakerIO{
    private static final TalonFX motor = new TalonFX(IntakerConstants.INTAKE_MOTOR_ID, "rio");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public IntakerIOPhoenix6(){
        motorConfig();
    }
    
    public void motorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = IntakerConstants.INTAKE_RATIO;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = IntakerConstants.K_P;
        config.Slot0.kI = IntakerConstants.K_I;
        config.Slot0.kD = IntakerConstants.K_D;
        config.Slot0.kV = IntakerConstants.K_V;
        config.Slot0.kS = IntakerConstants.K_S;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotorOutput.Inverted = IntakerConstants.INVERTED;
        motor.getConfigurator().apply(config);


        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getVelocity(), motor.getPosition(), motor.getDeviceTemp()
        );
    }

    
    @Override
    public void setIntakeVoltage(double Voltage) {
        motor.setVoltage(Voltage);
    }


    @Override
    public void setIntakeRPS(double RPS) {
        if(RPS == 0){
            motor.stopMotor();
        }

        motor.setControl(dutycycle.withVelocity(RPS));
    }
    
    
 
    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        inputs.MotorConnected = BaseStatusSignal.refreshAll(
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getVelocity(),
            motor.getPosition(),
            motor.getDeviceTemp()
        ).isOK();
        
        inputs.VoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.CurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
		inputs.VelocityRPS = motor.getVelocity().getValueAsDouble();
		
	}
}
