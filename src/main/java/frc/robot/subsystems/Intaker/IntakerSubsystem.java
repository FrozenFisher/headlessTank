package frc.robot.subsystems.Intaker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakerConstants;
import frc.robot.subsystems.Intaker.IntakerIOInputsAutoLogged;


public class IntakerSubsystem extends SubsystemBase{
    public static IntakerSubsystem m_instance;
    public static IntakerSubsystem getInstance() {
        return m_instance == null? m_instance = new IntakerSubsystem() : m_instance;
    }

    public final IntakerIO io;
    public final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private double targetRPS = 0.;
    public IntakerSubsystem(){
        if(Robot.isReal()){
            io = new IntakerIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new IntakerIOPhoenix6();
        }
    }

    public void setVoltage(double velocity){
        io.setIntakeVoltage(velocity);
    }

    public void setRPS(double rps){
        targetRPS = rps;
        io.setIntakeRPS(rps);
    }

    public boolean IsAtTargetRPS(){
        return MathUtil.isNear(targetRPS, inputs.VelocityRPS, IntakerConstants.INTAKE_VELOCITY_TOLERANCE_RPS);
    }

    public void stop(){
        io.setIntakeRPS(0.);
    }
    

}
