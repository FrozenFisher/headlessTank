package frc.robot.subsystems.Tank;

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
    default public void setLeftVelocityMps(double leftMps){}
    default public void setRightVelocityMps(double rightMps){}
    default public void setVelocityMps(double leftMps, double rightMps){
        setLeftVelocityMps(leftMps);
        setRightVelocityMps(rightMps);
    }
    
    default public void resetWheelPositions(){}

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


        public double leftPositionMeters;
        public double rightPositionMeters;

    }
    default public void updateInputs(TankIOInputs inputs){}
}
    
