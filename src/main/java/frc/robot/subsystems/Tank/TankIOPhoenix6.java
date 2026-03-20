package frc.robot.subsystems.Tank;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.TankConstants;

public class TankIOPhoenix6 implements TankIO{
    private static final TalonFX leftMotor = new TalonFX(TankConstants.TANK_LEFT_MOTOR_ID, "rio");
    private static final TalonFX rightMotor = new TalonFX(TankConstants.TANK_RIGHT_MOTOR_ID, "rio");
    private static final Pigeon2 pigeon = new Pigeon2(TankConstants.TANK_PIGEON_ID, "rio");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public TankIOPhoenix6(){
        motorConfig();
        pigeonConfig();
    }
    
    public void motorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = TankConstants.TANK_RATIO;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = TankConstants.K_P;
        config.Slot0.kI = TankConstants.K_I;
        config.Slot0.kD = TankConstants.K_D;
        config.Slot0.kV = TankConstants.K_V;
        config.Slot0.kS = TankConstants.K_S;

        config.MotorOutput.Inverted = TankConstants.LEFT_INVERTED;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = TankConstants.RIGHT_INVERTED;
        rightMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leftMotor.getMotorVoltage(), leftMotor.getSupplyCurrent(), leftMotor.getVelocity(), leftMotor.getPosition(), leftMotor.getDeviceTemp(),
            rightMotor.getMotorVoltage(), rightMotor.getSupplyCurrent(), rightMotor.getVelocity(), rightMotor.getPosition(), rightMotor.getDeviceTemp()
        );
    }

    public void pigeonConfig(){
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pigeon.getYaw(), pigeon.getPitch(), pigeon.getRoll(), pigeon.getAngularVelocityZWorld()
        );
    }
    
    @Override
    public void setLeftVoltage(double leftVoltage) {
        leftMotor.setVoltage(leftVoltage);
    }
    public void setRightVoltage(double rightVoltage) {
        rightMotor.setVoltage(rightVoltage);
    }

    @Override
    public void setLeftRPS(double leftRPS) {
        if(leftRPS == 0){
            leftMotor.stopMotor();
        }

        leftMotor.setControl(dutycycle.withVelocity(leftRPS));
    }
    
    @Override
    public void setRightRPS(double rightRPS) {
        if(rightRPS == 0){
            rightMotor.stopMotor();
        }

        rightMotor.setControl(dutycycle.withVelocity(rightRPS));
    }

    @Override
    public void setRPS(double leftRPS, double rightRPS) {
        if(leftRPS == 0 && rightRPS == 0){
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            leftMotor.setControl(dutycycle.withVelocity(leftRPS));
            rightMotor.setControl(dutycycle.withVelocity(rightRPS));
        }
    }

    private static double metersPerSecondToRps(double mps){
        return mps / TankConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    @Override
    public void setLeftVelocityMps(double leftMps){
        setLeftRPS(metersPerSecondToRps(leftMps));
    }
    @Override
    public void setRightVelocityMps(double rightMps){
        setRightRPS(metersPerSecondToRps(rightMps));
    }
    @Override
    public void setVelocityMps(double leftMps, double rightMps){
        setRPS(metersPerSecondToRps(leftMps), metersPerSecondToRps(rightMps));
    }

    @Override
    public void resetHeading() {
        pigeon.reset();
    }

    @Override
    public void setHeading(double heading) {
        pigeon.setYaw(heading);
    }

    @Override
    public void resetWheelPositions(){
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }
 
    @Override
    public void updateInputs(TankIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
            leftMotor.getMotorVoltage(),
            leftMotor.getSupplyCurrent(),
            leftMotor.getVelocity(),
            leftMotor.getPosition(),
            leftMotor.getDeviceTemp()
        ).isOK();
        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
            rightMotor.getMotorVoltage(),
            rightMotor.getSupplyCurrent(),
            rightMotor.getVelocity(),
            rightMotor.getPosition(),
            rightMotor.getDeviceTemp()
        ).isOK();
        inputs.pigeonConnected = BaseStatusSignal.refreshAll(
            pigeon.getYaw(),
            pigeon.getPitch(),
            pigeon.getRoll(),
            pigeon.getAngularVelocityZWorld()
        ).isOK();
        
        inputs.leftVoltageVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightVoltageVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rightCurrentAmps = rightMotor.getSupplyCurrent().getValueAsDouble();

		inputs.leftVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
		inputs.rightVelocityRPS = rightMotor.getVelocity().getValueAsDouble();
		inputs.leftVelocityMps = inputs.leftVelocityRPS * TankConstants.TANK_RATIO * TankConstants.WHEEL_CIRCUMFERENCE_METERS;
		inputs.rightVelocityMps = inputs.rightVelocityRPS * TankConstants.TANK_RATIO * TankConstants.WHEEL_CIRCUMFERENCE_METERS;//若负责移动的电机有传动系统，填入传动比
		
		double leftRotations = leftMotor.getPosition().getValueAsDouble();
		double rightRotations = rightMotor.getPosition().getValueAsDouble();
		inputs.leftPositionMeters = leftRotations * TankConstants.WHEEL_CIRCUMFERENCE_METERS * TankConstants.TANK_RATIO;
		inputs.rightPositionMeters = rightRotations * TankConstants.WHEEL_CIRCUMFERENCE_METERS * TankConstants.TANK_RATIO;

		// Pigeon姿态
		inputs.headingDegrees = pigeon.getYaw().getValueAsDouble();
		inputs.pitchDegrees = pigeon.getPitch().getValueAsDouble();
		inputs.rollDegrees = pigeon.getRoll().getValueAsDouble();
		inputs.angularVelocityZ = pigeon.getAngularVelocityZWorld().getValueAsDouble();
	}
}
