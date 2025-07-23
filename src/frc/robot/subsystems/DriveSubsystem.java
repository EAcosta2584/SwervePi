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
import robotCore.Logger;
import robotCore.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Steering;

public class DriveSubsystem extends SubsystemBase {

    SwerveModule m_frontLeft = new SwerveModule(Swerve.FLDrivePWM, Swerve.FLDriveDir, Swerve.FLDriveEncInt, 
        Swerve.FLDriveEncDir, Swerve.FLSteeringPWM, Swerve.FLSteeringDir, Swerve.FLSteeringEncA, Swerve.FLSteeringEncB, Swerve.FLI2CAddr, "Front Left");
    SwerveModule m_frontRight = new SwerveModule(Swerve.FRDrivePWM, Swerve.FRDriveDir, Swerve.FRDriveEncInt, 
        Swerve.FRDriveEncDir, Swerve.FRSteeringPWM, Swerve.FRSteeringDir, Swerve.FRSteeringEncA, Swerve.FRSteeringEncB, Swerve.FRI2CAddr, "Front Right");
    SwerveModule m_backLeft = new SwerveModule(Swerve.BLDrivePWM, Swerve.BLDriveDir, Swerve.BLDriveEncInt, 
        Swerve.BLDriveEncDir, Swerve.BLSteeringPWM, Swerve.BLSteeringDir, Swerve.BLSteeringEncA, Swerve.BLSteeringEncB, Swerve.BLI2CAddr, "Back Left");
    SwerveModule m_backRight = new SwerveModule(Swerve.BRDrivePWM, Swerve.BRDriveDir, Swerve.BRDriveEncInt, 
        Swerve.BRDriveEncDir, Swerve.BRSteeringPWM, Swerve.BRSteeringDir, Swerve.BRSteeringEncA, Swerve.BRSteeringEncB, Swerve.BRI2CAddr, "Back Right");

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

    @Override
    public void periodic(){
        Logger.log("DriveSubsystem", -1, "periodic()");
        //Logger.log("FLPos", 3, String.valueOf(GetFrontLeftModule().getSteeringPositionInDegrees()));
        //Logger.log("FRPos", 3, String.valueOf(GetFrontRightModule().getSteeringPositionInDegrees()));
        //Logger.log("BLPos", 3, String.valueOf(GetBackLeftModule().getSteeringPositionInDegrees()));
        //Logger.log("BRPos", 3, String.valueOf(GetBackRightModule().getSteeringPositionInDegrees()));
    }
}
