package frc.robot.subsystems.TankLike2910;

import org.littletonrobotics.junction.AutoLog;

public interface TankIO {
    default public void setLeftVoltage(double leftVoltage){}
    default public void setRightVoltage(double rightVoltage){}
    default public void setLeftRPS(double leftRPS){}
    default public void setRightRPS(double rightRPS){}
    default public void setRPS(double leftRPS, double rightRPS) {
        setLeftRPS(leftRPS);
        setRightRPS(rightRPS);
    }
    // New: velocity control in meters per second
    default public void setLeftVelocityMps(double leftMps){}
    default public void setRightVelocityMps(double rightMps){}
    default public void setVelocityMps(double leftMps, double rightMps){
        setLeftVelocityMps(leftMps);
        setRightVelocityMps(rightMps);
    }
    /** Enable/disable brake mode if supported. */
    default public void setBrakeMode(boolean enable) {}

    @AutoLog
    public class TankIOInputs {
        public boolean leftMotorConnected;
        public boolean rightMotorConnected;

        public double leftVoltageVolts;
        public double rightVoltageVolts;
        public double leftCurrentAmps;
        public double rightCurrentAmps;
        public double leftVelocityRPS;
        public double rightVelocityRPS;
        public double leftVelocityMps;
        public double rightVelocityMps;
        // Added telemetry similar to 2910 style
        public double leftPositionRotations;
        public double rightPositionRotations;
        public double leftTemperatureCelsius;
        public double rightTemperatureCelsius;

    }
    default public void updateInputs(TankIOInputs inputs){}






    
}
    
