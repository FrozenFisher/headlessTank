package frc.robot.subsystems.Intaker;

import org.littletonrobotics.junction.AutoLog;

public interface IntakerIO {
    default public void setIntakeVoltage(double voltage){}
    default public void setIntakeRPS(double rps){}

    
    @AutoLog
    public class IntakerIOInputs {
        public boolean MotorConnected;

        public double VoltageVolts;
        public double CurrentAmps;
        public double VelocityRPS;

    }
    default public void updateInputs(IntakerIOInputs inputs){}
    
} 