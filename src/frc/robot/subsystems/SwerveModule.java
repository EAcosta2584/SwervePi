package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;
import robotCore.Encoder;
import robotCore.Gyro;

public class SwerveModule {
    private final PWMMotor m_driveMotor;
    private final PWMMotor m_steeringMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_steeringEncoder;

    private final double k_degreesPerTick;
    private final double k_deadZone = 2.0;

    private final Gyro m_Gyro = new Gyro();

    public SwerveModule(int drivePWM, int driveDir, int driveEncInt, int driveEncDir, 
                        int steeringPWM, int steeringDir, int steeringEncA, int steeringEncB, 
                        int i2cAddr, String name) {

        m_driveMotor = new PWMMotor(drivePWM, driveDir, i2cAddr);
        m_steeringMotor = new PWMMotor(steeringPWM, steeringDir, i2cAddr);
        m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, steeringEncA, steeringEncB, i2cAddr, true);
        m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, driveEncInt, driveEncDir, i2cAddr, true);

        m_driveMotor.setFeedbackDevice(m_driveEncoder);
        m_steeringMotor.setFeedbackDevice(m_steeringEncoder);

        k_degreesPerTick = 360.0 / m_steeringEncoder.getRange(); //Converts arbitrary rot sensor to degrees
    
        m_steeringMotor.setDeadZone(k_deadZone / k_degreesPerTick);

        //m_steeringMotor.setInverted(true);
    }

    // Drive Setters
    public void setDriveMinPower(double power){
        m_driveMotor.setMinPower(power);
    }

    public void setDrivePower(double power){
        m_driveMotor.setControlMode(SmartMotorMode.Power);
        m_driveMotor.set(power);
    }

    public void setDriveSpeed(double speed){
        m_driveMotor.setControlMode(SmartMotorMode.Speed);
        m_driveMotor.set(speed);
    }

    public void setDriveSpeedMPS(double speed){
        m_driveMotor.set(speed * Constants.Drive.k_ticksPerMeter);
    }

    // Drive Getters
    public double getDriveSpeed(){
        return m_driveEncoder.getSpeed();
    }

    public double getDrivePosition(){
        return m_driveEncoder.getPosition();
    }

    public double getDriveSpeedMPS(){
        return getDriveSpeed() / Constants.Drive.k_ticksPerMeter;
    }

    // Steering Setters
    public void setSteeringZero(int zero){
        m_steeringEncoder.setZero(zero);
    }

    public void setSteeringPTerm(double p){
        m_steeringMotor.setPTerm(p);
    }

    public void setSteeringDTerm(double d){
        m_steeringMotor.setDTerm(d);
    }

    public void setSteeringMinPower(double power){
        m_steeringMotor.setMinPower(power);
    }

    public void setSteeringPower(double power){
        m_steeringMotor.setControlMode(SmartMotorMode.Power);
        m_steeringMotor.set(power);
    }

    public void setSteeringPosition(double angleInDegrees){
        m_steeringMotor.setControlMode(SmartMotorMode.Position);
        m_steeringMotor.set(angleInDegrees/k_degreesPerTick);
    }

    // Steering Getters
    public double getSteeringPosition() {
        return m_steeringEncoder.getPosition();
    }

    public double getSteeringPositionInDegrees(){
        return Gyro.normalizeAngle(getSteeringPosition()*k_degreesPerTick);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
 
        Rotation2d encoderRotation = Rotation2d.fromDegrees(getSteeringPosition());
 
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
 
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();
 
        setDriveSpeedMPS(state.speedMetersPerSecond);
        setSteeringPosition(state.angle.getDegrees());
    }


    // Stop Motors
    public void stopSteering() {
        setSteeringPower(0);
    }

    public void stopDrive() {
        setDrivePower(0);
    }

    public void stop() {
        stopSteering();
        stopDrive();
    }
        
}
