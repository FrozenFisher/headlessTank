package frc.robot.subsystems.Tank;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TankConstants;
import frc.robot.commands.DriveCommand.ManualDriveCommand;
import frc.robot.subsystems.Tank.TankIOPhoenix6;

public class TankSubsystem extends SubsystemBase{
    public static TankSubsystem m_instance;
    public static TankSubsystem getInstance() {
        return m_instance == null? m_instance = new TankSubsystem() : m_instance;
    }

    public final TankIO io;
    public final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

    private double targetRPSLeft = 0.;
    private double targetRPSRight = 0.;
    private double targetMpsLeft = 0.;
    private double targetMpsRight = 0.;
    private double targetHeading = 0.0;
    private PIDController turnPIDController;
    private double sendedSpeed = 0.;


    private DifferentialDriveOdometry odometry;
    private Pose2d currentPose = new Pose2d();

    private boolean shouldMoveForward = true;
    private Pose2d targetPose = new Pose2d();
    private PIDController positionPID = new PIDController(0.05, 0, 0); // TUNE these values

    public TankSubsystem(){
        if(Robot.isReal()){
            io = new TankIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new TankIOPhoenix6();
        }
        
        // Init Turning PIDController
        turnPIDController = new PIDController(
            TankConstants.HeadlessControlConstants.TURN_K_P,
            TankConstants.HeadlessControlConstants.TURN_K_I,
            TankConstants.HeadlessControlConstants.TURN_K_D
        );
        
        // Use 0..360 domain for headings
        turnPIDController.enableContinuousInput(0.0, 360.0);//Input Range
        turnPIDController.setTolerance(2.0); //容差
        
        // Use corrected heading for odometry initialization
        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(correctedHeadingDegrees()),
            inputs.leftPositionMeters,
            inputs.rightPositionMeters,
            new Pose2d()
        );
    }

    // Move
    public void setRPSLeft(double rps){
        targetRPSLeft = rps;
        io.setLeftRPS(rps);
    }
    public void setRPSRight(double rps){
        targetRPSRight = rps;
        io.setRightRPS(rps);
    }
    public void setRPS(double leftRPS, double rightRPS){
        targetRPSLeft = leftRPS;
        targetRPSRight = rightRPS;
        io.setRPS(leftRPS, rightRPS);
    }
    public void setVelocityLeft(double mps){
        targetMpsLeft = mps;
        io.setLeftVelocityMps(mps);
    }
    public void setVelocityRight(double mps){
        targetMpsRight = mps;
        io.setRightVelocityMps(mps);
    }
    public void setVelocity(double leftMps, double rightMps){
        targetMpsLeft = leftMps;
        targetMpsRight = rightMps;
        io.setVelocityMps(leftMps, rightMps);
    }
    
    public boolean IsAtTargetRPSLeft(){
        return MathUtil.isNear(targetRPSLeft, inputs.leftVelocityRPS, TankConstants.TANK_VELOCITY_TOLERANCE_RPS);
    }
    public boolean IsAtTargetRPSRight(){
        return MathUtil.isNear(targetRPSRight, inputs.rightVelocityRPS, TankConstants.TANK_VELOCITY_TOLERANCE_RPS);
    }
    public boolean IsAtTargetRPSBoth(){
        return IsAtTargetRPSLeft() && IsAtTargetRPSRight();
    }
    public boolean IsAtTargetMpsLeft(){
        return MathUtil.isNear(targetMpsLeft, inputs.leftVelocityMps, TankConstants.TANK_VELOCITY_TOLERANCE_MPS);
    }
    public boolean IsAtTargetMpsRight(){
        return MathUtil.isNear(targetMpsRight, inputs.rightVelocityMps, TankConstants.TANK_VELOCITY_TOLERANCE_MPS);
    }
    public boolean IsAtTargetMpsBoth(){
        return IsAtTargetMpsLeft() && IsAtTargetMpsRight();
    }
    
    public void stopLeft(){
        setRPSLeft(0.);
    }
    public void stopRight(){
        setRPSRight(0.);
    }
    public void stopBoth(){
        setVelocity(0., 0.);
    }

    public void setVoltageLeft(double voltage){
        io.setLeftVoltage(voltage);
    }
    public void setVoltageRight(double voltage){
        io.setRightVoltage(voltage);
    }

    // Turn
    public void turnToHeading(double targetHeadingDegrees) {
        // targetHeading is expressed in 0..360 degrees
        targetHeading = targetHeadingDegrees;
        double turnOutput = calculateTurnOutput();
        setVelocity(turnOutput, -turnOutput);
    }

    private double calculateTurnOutput() {
        // PID works with corrected heading
        double output = turnPIDController.calculate(correctedHeadingDegrees(), targetHeading);
        output = MathUtil.clamp(output, -TankConstants.K_TURN_MPS, TankConstants.K_TURN_MPS);
        return output;
    }

    public boolean isAtTargetHeading() {
        return turnPIDController.atSetpoint();
    }

    public boolean isAtTargetHeading(double tolerance) {
        return Math.abs(turnPIDController.getPositionError()) < tolerance;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    // Pigeon
    public void resetHeading() {
        io.setHeading(0.);;
        targetHeading = 0.0;
        shouldMoveForward = true;
        turnPIDController.reset();
    }

    public void setHeading(double heading) {
        io.setHeading(heading);
        targetHeading = heading;
        turnPIDController.reset();
    }

    public double getHeading() {
        // Return heading in 0..360 domain (after offset)
        return correctedHeadingDegrees();
    }

    /**
     * Returns the heading after applying the team-configurable offset so that
     * the reported heading matches the coordinate frame expected by controls.
     */
    public double correctedHeadingDegrees() {
        double raw = inputs.headingDegrees;
        // Apply configured sensor offset then normalize into 0..360
        double corrected = raw + TankConstants.HeadlessControlConstants.HEADING_OFFSET_DEGREES;
        return corrected;
    }

    public double getPitch() {
        return inputs.pitchDegrees;
    }

    public double getRoll() {
        return inputs.rollDegrees;
    }

    public double getAngularVelocityZ() {
        return inputs.angularVelocityZ;
    }

    public boolean isPigeonConnected() {
        return inputs.pigeonConnected;
    }

    public Pose2d getPose(){ 
        return currentPose; 
    }
    
    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(Rotation2d.fromDegrees(correctedHeadingDegrees()), inputs.leftPositionMeters, inputs.rightPositionMeters, pose);
        currentPose = pose;
    }

    // driveControl
    //正常差速驱动
    public void arcadedrive(double forward, double turn) {
        double leftMps = forward * TankConstants.K_FWD_MPS + turn * TankConstants.K_TURN_MPS;
        double rightMps = forward * TankConstants.K_FWD_MPS - turn * TankConstants.K_TURN_MPS;

        leftMps = MathUtil.clamp(leftMps, -TankConstants.K_MAX_SPEED_MPS, TankConstants.K_MAX_SPEED_MPS);
        rightMps = MathUtil.clamp(rightMps, -TankConstants.K_MAX_SPEED_MPS, TankConstants.K_MAX_SPEED_MPS);

        setVelocityLeft(leftMps);
        setVelocityRight(rightMps);
    }
    
    //无头模式移动
    public void headlessMove(double speed) {
        speed = MathUtil.clamp(speed, -TankConstants.K_MAX_SPEED_MPS, TankConstants.K_MAX_SPEED_MPS);
        sendedSpeed = speed;
        if (shouldMoveForward) {
            setVelocity(speed, speed);
        } else {
            setVelocity(-speed, -speed);
        }
        
    }
    
    //无头模式转向
    public boolean headlessTurn(double targetAngle, boolean isLeftStickInput) {
        double finalTargetAngle = targetAngle;
        
        if (isLeftStickInput) {
            // 注释掉最短转向角度的计算，直接转向 stick 指向的角度
            // double currentHeading = getHeading();
            // double forwardAngleDiff = MathUtil.inputModulus(targetAngle, 0., 360.) - currentHeading;
            // double backwardAngleDiff = MathUtil.inputModulus(targetAngle+180., 0., 360.)  - currentHeading;
            // if (Math.abs(forwardAngleDiff) <= Math.abs(backwardAngleDiff)) {
            //     finalTargetAngle = targetAngle;
            //     if (forwardAngleDiff > 10.){
            //         shouldMoveForward = true;
            //     }
            // } else {
            //     if (backwardAngleDiff > 10.){
            //         shouldMoveForward = false;
            //     }
            //     finalTargetAngle = MathUtil.inputModulus(targetAngle+180., 0., 360.) ;
            // }
            finalTargetAngle = targetAngle;
            shouldMoveForward = true;
        } else {
            shouldMoveForward = true;
        }

        // 直接转向 stick 指向的角度
        turnToHeading(finalTargetAngle);
        
        return isAtTargetHeading(5.);
    }

    public void setTargetPoint(Pose2d target) {
        this.targetPose = target;
    }
    
    public boolean driveToPoint(Pose2d targetPose) {
        setTargetPoint(targetPose);
        Pose2d currentPose = getPose();
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleToTarget = Math.toDegrees(Math.atan2(
            targetPose.getTranslation().getY() - currentPose.getTranslation().getY(),
            targetPose.getTranslation().getX() - currentPose.getTranslation().getX()
        ));
        
        // First turn to face the target
        if (!isAtTargetHeading(5.0)) {
            turnToHeading(angleToTarget);
            return false;
        }
        
        // Then move towards target
        double speed = MathUtil.clamp(
            positionPID.calculate(0, distance),
            -TankConstants.K_MAX_SPEED_MPS,
            TankConstants.K_MAX_SPEED_MPS
        );
        
        setVelocity(speed, speed);
        
        // Check if reached target
        return distance < 0.1; // TUNE this tolerance
    }
    
    //停止所有运动
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
        
        // Update odometry with corrected heading
        currentPose = odometry.update(
            Rotation2d.fromDegrees(correctedHeadingDegrees()),
            inputs.leftPositionMeters,
            inputs.rightPositionMeters
        );
        
        processLog();
        processDashboard();
    }

    private void processLog(){
        Logger.processInputs("Tank", inputs);
        Logger.recordOutput("Tank/TargetRPSLeft", targetRPSLeft);
        Logger.recordOutput("Tank/TargetRPSRight", targetRPSRight);
        Logger.recordOutput("Tank/IsAtTargetRPS", IsAtTargetRPSBoth());
        Logger.recordOutput("Tank/TargetMpsLeft", targetMpsLeft);
        Logger.recordOutput("Tank/TargetMpsRight", targetMpsRight);
        Logger.recordOutput("Tank/IsAtTargetMps", IsAtTargetMpsBoth());
        
        Logger.recordOutput("Tank/Heading", getHeading());
        Logger.recordOutput("Tank/Pitch", getPitch());
        Logger.recordOutput("Tank/Roll", getRoll());
        Logger.recordOutput("Tank/AngularVelocityZ", getAngularVelocityZ());
        Logger.recordOutput("Tank/PigeonConnected", isPigeonConnected());
        Logger.recordOutput("Tank/TargetHeading", targetHeading);
        Logger.recordOutput("Tank/TurnOutput", calculateTurnOutput());
        Logger.recordOutput("Tank/IsAtTargetHeading", isAtTargetHeading());
        Logger.recordOutput("Tank/shouldMoveForward", shouldMoveForward);
        Logger.recordOutput("Tank/Speed", sendedSpeed);

        Logger.recordOutput("Tank/Pose", currentPose);
    }

    private void processDashboard(){
        //TODO: Implement dashboard code here
    }
}







    
