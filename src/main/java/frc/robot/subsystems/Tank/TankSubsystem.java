package frc.robot.subsystems.Tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class TankSubsystem extends SubsystemBase {
  public static TankSubsystem m_instance;

  public static TankSubsystem getInstance() {
    return m_instance == null ? m_instance = new TankSubsystem() : m_instance;
  }

  public final TankIO io;
  public final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

  private double targetRPSLeft = 0.;
  private double targetRPSRight = 0.;
  private double targetMpsLeft = 0.;
  private double targetMpsRight = 0.;
  private String isTurningTo = "Stopped";

  private final SlewRateLimiter arcadeForwardSlew =
      new SlewRateLimiter(TankConstants.kArcadeSlewRateMPS);
  private final SlewRateLimiter arcadeTurnSlew =
      new SlewRateLimiter(TankConstants.kArcadeSlewRateMPS);

  public TankSubsystem() {
    if (Robot.isReal()) {
      io = new TankIOPhoenix6();
    } else {
      // TODO: Implement simulation code here
      io = new TankIOPhoenix6();
    }
  }

  // Move
  public void setRPSLeft(double rps) {
    targetRPSLeft = rps;
    io.setLeftRPS(rps);
  }

  public void setRPSRight(double rps) {
    targetRPSRight = rps;
    io.setRightRPS(rps);
  }

  public void setRPS(double leftRPS, double rightRPS) {
    targetRPSLeft = leftRPS;
    targetRPSRight = rightRPS;
    io.setRPS(leftRPS, rightRPS);
  }

  public void setVelocityLeft(double mps) {
    targetMpsLeft = mps;
    io.setLeftVelocityMps(mps);
  }

  public void setVelocityRight(double mps) {
    targetMpsRight = mps;
    io.setRightVelocityMps(mps);
  }

  public void setVelocity(double leftMps, double rightMps) {
    targetMpsLeft = leftMps;
    targetMpsRight = rightMps;
    io.setVelocityMps(leftMps, rightMps);
  }

  public boolean IsAtTargetRPSLeft() {
    return MathUtil.isNear(
        targetRPSLeft, inputs.leftVelocityRPS, TankConstants.TankVelocityToleranceRPS);
  }

  public boolean IsAtTargetRPSRight() {
    return MathUtil.isNear(
        targetRPSRight, inputs.rightVelocityRPS, TankConstants.TankVelocityToleranceRPS);
  }

  public boolean IsAtTargetRPSBoth() {
    return IsAtTargetRPSLeft() && IsAtTargetRPSRight();
  }

  public boolean IsAtTargetMpsLeft() {
    return MathUtil.isNear(
        targetMpsLeft, inputs.leftVelocityMps, TankConstants.TankVelocityToleranceMPS);
  }

  public boolean IsAtTargetMpsRight() {
    return MathUtil.isNear(
        targetMpsRight, inputs.rightVelocityMps, TankConstants.TankVelocityToleranceMPS);
  }

  public boolean IsAtTargetMpsBoth() {
    return IsAtTargetMpsLeft() && IsAtTargetMpsRight();
  }

  public void stopLeft() {
    setRPSLeft(0.);
  }

  public void stopRight() {
    setRPSRight(0.);
  }

  public void stopBoth() {
    setVelocity(0., 0.);
    arcadeForwardSlew.reset(0.);
    arcadeTurnSlew.reset(0.);
  }

  public void setVoltageLeft(double voltage) {
    io.setLeftVoltage(voltage);
  }

  public void setVoltageRight(double voltage) {
    io.setRightVoltage(voltage);
  }

  // driveControl
  // 正常差速驱动
  public void arcadedrive(double forward, double turn) {
    forward = forward * TankConstants.kForwardMPS;
    turn = turn * TankConstants.kForwardMPS;

    forward = arcadeForwardSlew.calculate(forward);
    turn = arcadeTurnSlew.calculate(turn);

    double leftMps = forward + turn;
    double rightMps = forward - turn;

    leftMps = MathUtil.clamp(leftMps, -TankConstants.kMaxSpeedMPS, TankConstants.kMaxSpeedMPS);
    rightMps = MathUtil.clamp(rightMps, -TankConstants.kMaxSpeedMPS, TankConstants.kMaxSpeedMPS);

    targetMpsLeft = leftMps;
    targetMpsRight = rightMps;
    io.setVelocityMps(leftMps, rightMps);
  }

  // 停止所有运动
  public void stop() {
    stopBoth();
  }

  public void stop(String side) {
    switch (side) {
      case "left":
        stopLeft();
        break;
      case "right":
        stopRight();
        break;
      case "both":
        stopBoth();
        break;
      default:
        stopBoth();
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processLog();
    processDashboard();
  }

  private void processLog() {
    Logger.processInputs("Tank", inputs);
    Logger.recordOutput("Tank/TargetRPSLeft", targetRPSLeft);
    Logger.recordOutput("Tank/TargetRPSRight", targetRPSRight);
    Logger.recordOutput("Tank/IsAtTargetRPS", IsAtTargetRPSBoth());
    Logger.recordOutput("Tank/TargetMpsLeft", targetMpsLeft);
    Logger.recordOutput("Tank/TargetMpsRight", targetMpsRight);
    Logger.recordOutput("Tank/IsAtTargetMps", IsAtTargetMpsBoth());
  }

  private void processDashboard() {
    // TODO: Implement dashboard code here
  }



  public void IsTurningTo() {
    if (inputs.leftVelocityMps > 0 && inputs.rightVelocityMps > 0){
      if (inputs.leftVelocityMps > inputs.rightVelocityMps) {
        Logger.recordOutput("Tank/IsTurning", "Left");
        isTurningTo = "Left";
      } else if (inputs.leftVelocityMps < inputs.rightVelocityMps) {
        Logger.recordOutput("Tank/IsTurning", "Right");
        isTurningTo = "Right";
      } else {
        Logger.recordOutput("Tank/IsTurning", "Straight");
        isTurningTo = "Straight";
      }
    }else
    {
      Logger.recordOutput("Tank/IsTurning", "Stopped");
      isTurningTo = "Stopped";
    }
  }
  public String getIsTurningTo() {
    return isTurningTo;
  }
}
