package frc.robot.subsystems.TankLike2910;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.Tank.TankIO.TankIOInputs;




public class TankSubsystem extends SubsystemBase{
    public static TankSubsystem m_instance;
    public static TankSubsystem getInstance(CommandXboxController controller) {
        return m_instance == null? m_instance = new TankSubsystem(controller) : m_instance;
    }

    public final TankIO io;
    public final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

    private final CommandXboxController controller;
    private final DoubleSupplier leftStickYSupplier;
    private final DoubleSupplier rightStickXSupplier;

    public enum WantedState {
        SYS_ID,
        TELEOP_DRIVE,
        DRIVE_2_POINT,
        PATH_PLANNER,
        IDLE
    }

    public enum SystemState {
        SYS_ID,
        TELEOP_DRIVE,
        DRIVE_2_POINT,
        PATH_PLANNER,
        IDLE
    }

    private SystemState systemState = SystemState.TELEOP_DRIVE;
    private WantedState wantedState = WantedState.TELEOP_DRIVE;
    // private SystemState systemState = SystemState.SYS_ID;
    // private WantedState wantedState = WantedState.SYS_ID;

    private double targetRPSLeft = 0.;
    private double targetRPSRight = 0.;
    private double targetMpsLeft = 0.;
    private double targetMpsRight = 0.;

    public TankSubsystem(CommandXboxController controller){
        this.controller = controller;
        if(Robot.isReal()){
            io = new TankIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new TankIOPhoenix6();
        }
        this.leftStickYSupplier = () -> this.controller.getLeftY();
        this.rightStickXSupplier = () -> this.controller.getRightX();
    }

    //state machine

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case SYS_ID -> SystemState.SYS_ID;
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case DRIVE_2_POINT -> SystemState.DRIVE_2_POINT;
            default -> SystemState.IDLE;
        };
    }

    private void applyStates() {
        switch (systemState) {
            default:
            case SYS_ID:
                break;
            case TELEOP_DRIVE: {
                double forward = Math.pow(MathUtil.applyDeadband(leftStickYSupplier.getAsDouble(), TankConstants.K_DEADBAND), TankConstants.K_INPUT_POW);
                double turn = Math.pow(MathUtil.applyDeadband(rightStickXSupplier.getAsDouble(), TankConstants.K_DEADBAND), TankConstants.K_INPUT_POW);
                double leftMps = forward * TankConstants.K_FWD_MPS + turn * TankConstants.K_TURN_MPS;
                double rightMps = forward * TankConstants.K_FWD_MPS - turn * TankConstants.K_TURN_MPS;

                leftMps = MathUtil.clamp(leftMps, -TankConstants.K_MAX_SPEED_MPS, TankConstants.K_MAX_SPEED_MPS);
                rightMps = MathUtil.clamp(rightMps, -TankConstants.K_MAX_SPEED_MPS, TankConstants.K_MAX_SPEED_MPS);

                this.setVelocityLeft(leftMps);
                this.setVelocityRight(rightMps);
                break;
            }
            case PATH_PLANNER: {
                break;
            }
            case DRIVE_2_POINT: {
                break;
            }
            
        }
    }


    public void setState(WantedState state) {
        this.wantedState = state;
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    @Override
    public void periodic() {


        systemState = handleStateTransition();
        Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
        Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);
        applyStates();
        
        processLog();
        processDashboard();
    }



    //subsystem
    public void setRPSLeft(double rps){
        targetRPSLeft = rps;
        io.setLeftRPS(rps);
    }
    public void setRPSRight(double rps){
        targetRPSRight = rps;
        io.setRightRPS(rps);
    }
    public void setVelocityLeft(double mps){
        targetMpsLeft = mps;
        io.setLeftVelocityMps(mps);
    }
    public void setVelocityRight(double mps){
        targetMpsRight = mps;
        io.setRightVelocityMps(mps);
    }
    
    public boolean IsAtTargetRPSLeft(){
        return MathUtil.isNear(targetRPSLeft, inputs.leftVelocityRPS, TankConstants.TANK_VELOCITY_TOLERANCE_RPS);
    }
    public boolean IsAtTargetRPSRight(){
        return MathUtil.isNear(targetRPSRight, inputs.rightVelocityRPS, TankConstants.TANK_VELOCITY_TOLERANCE_RPS);
    }
    public boolean IsAtTargetMpsLeft(){
        return MathUtil.isNear(targetMpsLeft, inputs.leftVelocityMps, TankConstants.TANK_VELOCITY_TOLERANCE_MPS);
    }
    public boolean IsAtTargetMpsRight(){
        return MathUtil.isNear(targetMpsRight, inputs.rightVelocityMps, TankConstants.TANK_VELOCITY_TOLERANCE_MPS);
    }
    
    public void stopLeft(){
        setVelocityLeft(0.);
    }
    public void stopRight(){
        setVelocityRight(0.);
    }

    public void setVoltageLeft(double voltage){
        io.setLeftVoltage(voltage);
    }
    public void setVoltageRight(double voltage){
        io.setRightVoltage(voltage);
    }


    private void processLog(){
        io.updateInputs(inputs);
        Logger.processInputs("Tank", inputs);
        Logger.recordOutput("Tank/TargetRPSLeft", targetRPSLeft);
        Logger.recordOutput("Tank/TargetRPSRight", targetRPSRight);
        Logger.recordOutput("Tank/IsAtTargetRPS", IsAtTargetRPSLeft()&&IsAtTargetRPSRight());
        Logger.recordOutput("Tank/TargetMpsLeft", targetMpsLeft);
        Logger.recordOutput("Tank/TargetMpsRight", targetMpsRight);
        Logger.recordOutput("Tank/IsAtTargetMps", IsAtTargetMpsLeft()&&IsAtTargetMpsRight());
    }

    private void processDashboard(){
        //TODO: Implement dashboard code here
    }


}







    
