package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Steering;

public class DriveSubsystem extends SubsystemBase {

    private final Gyro m_gyro = new Gyro();

    SwerveModule m_frontLeft = new SwerveModule(Swerve.FLDrivePWM, Swerve.FLDriveDir, Swerve.FLDriveEncInt, 
        Swerve.FLDriveEncDir, Swerve.FLSteeringPWM, Swerve.FLSteeringDir, Swerve.FLSteeringEncA, Swerve.FLSteeringEncB, Swerve.FLI2CAddr, "Front Left");
    SwerveModule m_frontRight = new SwerveModule(Swerve.FRDrivePWM, Swerve.FRDriveDir, Swerve.FRDriveEncInt, 
        Swerve.FRDriveEncDir, Swerve.FRSteeringPWM, Swerve.FRSteeringDir, Swerve.FRSteeringEncA, Swerve.FRSteeringEncB, Swerve.FRI2CAddr, "Front Right");
    SwerveModule m_backLeft = new SwerveModule(Swerve.BLDrivePWM, Swerve.BLDriveDir, Swerve.BLDriveEncInt, 
        Swerve.BLDriveEncDir, Swerve.BLSteeringPWM, Swerve.BLSteeringDir, Swerve.BLSteeringEncA, Swerve.BLSteeringEncB, Swerve.BLI2CAddr, "Back Left");
    SwerveModule m_backRight = new SwerveModule(Swerve.BRDrivePWM, Swerve.BRDriveDir, Swerve.BRDriveEncInt, 
        Swerve.BRDriveEncDir, Swerve.BRSteeringPWM, Swerve.BRSteeringDir, Swerve.BRSteeringEncA, Swerve.BRSteeringEncB, Swerve.BRI2CAddr, "Back Right");

    // Robot wheel location
    private final Translation2d m_frontLeftLocation = new Translation2d(0.05842, 0.05842);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.05842, 0.05842);
    private final Translation2d m_backRightLocation = new Translation2d(-0.05842, -0.05842);
    private final Translation2d m_frontRightLocation = new Translation2d(0.05842, -0.05842);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_backLeftLocation, m_backRightLocation, m_frontRightLocation);


    

    public DriveSubsystem() {
        Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

        m_frontLeft.setSteeringMinPower(Steering.k_FLMinSteerPower);
        m_frontRight.setSteeringMinPower(Steering.k_FRMinSteerPower);
        m_backLeft.setSteeringMinPower(Steering.k_BLMinSteerPower);
        m_backRight.setSteeringMinPower(Steering.k_BRMinSteerPower);
        
        m_frontLeft.setSteeringZero(Steering.k_FLZero);
        m_frontRight.setSteeringZero(Steering.k_FRZero);
        m_backLeft.setSteeringZero(Steering.k_BLZero);
        m_backRight.setSteeringZero(Steering.k_BRZero);

        m_frontLeft.setSteeringPTerm(Steering.k_frontLeftSteeringP);
        m_frontRight.setSteeringPTerm(Steering.k_frontRightSteeringP);
        m_backLeft.setSteeringPTerm(Steering.k_backLeftSteeringP);
        m_backRight.setSteeringPTerm(Steering.k_backRightSteeringP);

        
        m_frontLeft.setSteeringDTerm(Steering.k_frontLeftSteeringD);
        m_frontRight.setSteeringDTerm(Steering.k_frontRightSteeringD);
        m_backLeft.setSteeringDTerm(Steering.k_backLeftSteeringD);
        m_backRight.setSteeringDTerm(Steering.k_backRightSteeringD);

        m_frontLeft.setDriveMinPower(Drive.k_FLminDrivePower);
        m_frontRight.setDriveMinPower(Drive.k_FRminDrivePower);
        m_backLeft.setDriveMinPower(Drive.k_BLminDrivePower);
        m_backRight.setDriveMinPower(Drive.k_BRminDrivePower);

        m_gyro.invert(false);
        m_gyro.reset(0);

    }

    public void resetGyro() {
        m_gyro.reset(0);
      }

    public SwerveModule GetFrontLeftModule(){
        return m_frontLeft;
    }

    public SwerveModule GetFrontRightModule(){
        return m_frontRight;
    }

    public SwerveModule GetBackLeftModule(){
        return m_backLeft;
    }

    public SwerveModule GetBackRightModule(){
        return m_backRight;
    }


    public void stop(){
        m_frontLeft.stop();
        m_backLeft.stop();
        m_backRight.stop();
        m_frontRight.stop();
  }

    private void setModuleStates(SwerveModuleState[] swerveModuleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.k_maxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_backLeft.setDesiredState(swerveModuleStates[1]);
        m_backRight.setDesiredState(swerveModuleStates[2]);
        m_frontRight.setDesiredState(swerveModuleStates[3]);
  }

    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
 
    setModuleStates(swerveModuleStates);
  }

    @Override
    public void periodic(){
        Logger.log("DriveSubsystem", -1, "periodic()");
        Logger.log("Yaw", 1, String.valueOf(m_gyro.getYaw()));
        Logger.log("Rotation", 1, String.valueOf(m_gyro.getRotation2d()));
        //Logger.log("DriveDist", 1, String.valueOf(GetFrontLeftModule().getDrivePosition()));
        //Logger.log("DriveDist", 1, String.valueOf(GetFrontRightModule().getDrivePosition()));
        //Logger.log("DriveDist", 1, String.valueOf(GetBackLeftModule().getDrivePosition()));
        //Logger.log("DriveDist", 1, String.valueOf(GetBackRightModule().getDrivePosition()));
        //Logger.log("FLPos", 3, String.valueOf(GetFrontLeftModule().getSteeringPosition()));
        //Logger.log("FRPos", 3, String.valueOf(GetFrontRightModule().getSteeringPositionInDegrees()));
        //Logger.log("BLPos", 3, String.valueOf(GetBackLeftModule().getSteeringPositionInDegrees()));
        //Logger.log("BRPos", 3, String.valueOf(GetBackRightModule().getSteeringPositionInDegrees()));
    }
}
