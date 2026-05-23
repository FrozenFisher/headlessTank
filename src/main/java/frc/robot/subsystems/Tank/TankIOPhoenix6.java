package frc.robot.subsystems.Tank;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.TankConstants;

public class TankIOPhoenix6 implements TankIO {
  private static final TalonFX leftMotor =
      new TalonFX(TankConstants.TankLeftMotorID, CANBus.roboRIO());
  private static final TalonFX leftFollowerMotor =
      new TalonFX(TankConstants.TankLeftFollowerID, CANBus.roboRIO());
  private static final Follower leftFollowerRequest =
      new Follower(TankConstants.TankLeftMotorID, TankConstants.TankLeftFollowerAlignment);

  private static final TalonFX rightMotor =
      new TalonFX(TankConstants.TankRightMotorID, CANBus.roboRIO());
  private static final TalonFX rightFollowerMotor =
      new TalonFX(TankConstants.TankRightFollowerID, CANBus.roboRIO());
  private static final Follower rightFollowerRequest =
      new Follower(TankConstants.TankRightMotorID, TankConstants.TankRightFollowerAlignment);

  private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

  public TankIOPhoenix6() {
    motorConfig();
  }

  public void motorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = TankConstants.TankRatio;
    config.Slot0.kP = TankConstants.kP;
    config.Slot0.kI = TankConstants.kI;
    config.Slot0.kD = TankConstants.kD;
    config.Slot0.kV = TankConstants.kV;
    config.Slot0.kS = TankConstants.kS;

    config.MotorOutput.Inverted = TankConstants.LeftInverted;
    leftMotor.getConfigurator().apply(config);
    leftFollowerMotor.getConfigurator().apply(config);
    leftFollowerMotor.setControl(leftFollowerRequest);

    config.MotorOutput.Inverted = TankConstants.RightInverted;
    rightMotor.getConfigurator().apply(config);
    rightFollowerMotor.getConfigurator().apply(config);
    rightFollowerMotor.setControl(rightFollowerRequest);
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
    if (leftRPS == 0) {
      leftMotor.stopMotor();
    }

    leftMotor.setControl(dutycycle.withVelocity(leftRPS));
  }

  @Override
  public void setRightRPS(double rightRPS) {
    if (rightRPS == 0) {
      rightMotor.stopMotor();
    }

    rightMotor.setControl(dutycycle.withVelocity(rightRPS));
  }

  @Override
  public void setRPS(double leftRPS, double rightRPS) {
    if (leftRPS == 0 && rightRPS == 0) {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    } else {
      leftMotor.setControl(dutycycle.withVelocity(leftRPS));
      rightMotor.setControl(dutycycle.withVelocity(rightRPS));
    }
  }

  private static double metersPerSecondToRps(double mps) {
    return mps / TankConstants.WheelCircumferenceMeters;
  }

  @Override
  public void setLeftVelocityMps(double leftMps) {
    setLeftRPS(metersPerSecondToRps(leftMps));
  }

  @Override
  public void setRightVelocityMps(double rightMps) {
    setRightRPS(metersPerSecondToRps(rightMps));
  }

  @Override
  public void setVelocityMps(double leftMps, double rightMps) {
    setRPS(metersPerSecondToRps(leftMps), metersPerSecondToRps(rightMps));
  }

  @Override
  public void resetWheelPositions() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  @Override
  public void updateInputs(TankIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftMotor.getMotorVoltage(),
                leftMotor.getSupplyCurrent(),
                leftMotor.getVelocity(),
                leftMotor.getPosition(),
                leftMotor.getDeviceTemp())
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightMotor.getMotorVoltage(),
                rightMotor.getSupplyCurrent(),
                rightMotor.getVelocity(),
                rightMotor.getPosition(),
                rightMotor.getDeviceTemp())
            .isOK();

    inputs.leftVoltageVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightVoltageVolts = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightCurrentAmps = rightMotor.getSupplyCurrent().getValueAsDouble();

    inputs.leftVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
    inputs.rightVelocityRPS = rightMotor.getVelocity().getValueAsDouble();
    inputs.leftVelocityMps =
        inputs.leftVelocityRPS * TankConstants.TankRatio * TankConstants.WheelCircumferenceMeters;
    inputs.rightVelocityMps =
        inputs.rightVelocityRPS * TankConstants.TankRatio * TankConstants.WheelCircumferenceMeters;

    double leftRotations = leftMotor.getPosition().getValueAsDouble();
    double rightRotations = rightMotor.getPosition().getValueAsDouble();
    inputs.leftPositionMeters =
        leftRotations * TankConstants.WheelCircumferenceMeters * TankConstants.TankRatio;
    inputs.rightPositionMeters =
        rightRotations * TankConstants.WheelCircumferenceMeters * TankConstants.TankRatio;
  }
}
