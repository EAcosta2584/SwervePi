package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;
import frc.robot.Constants;

public class CalibrateDist extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private double dist;
    private double m_vel;


    public CalibrateDist(DriveSubsystem subsystem){
        m_DriveSubsystem = subsystem;
        m_vel = Constants.Drive.k_FLminDrivePower;

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize(){

        m_DriveSubsystem.GetFrontLeftModule().setSteeringPosition(0);
        m_DriveSubsystem.GetFrontRightModule().setSteeringPosition(0);
        m_DriveSubsystem.GetBackLeftModule().setSteeringPosition(0);
        m_DriveSubsystem.GetBackRightModule().setSteeringPosition(0);
        
    }

    @Override
    public void execute(){
        m_DriveSubsystem.GetFrontLeftModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetFrontRightModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetBackLeftModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetBackRightModule().setDrivePower(m_vel);
        Logger.log("DriveDist", 1, String.valueOf(m_DriveSubsystem.GetFrontLeftModule().getDrivePosition()));
        Logger.log("DriveDist", 1, String.valueOf(m_DriveSubsystem.GetFrontRightModule().getDrivePosition()));
        Logger.log("DriveDist", 1, String.valueOf(m_DriveSubsystem.GetBackLeftModule().getDrivePosition()));
        Logger.log("DriveDist", 1, String.valueOf(m_DriveSubsystem.GetBackRightModule().getDrivePosition()));

    }

    @Override
    public void end(boolean interrupted){
        m_vel = 0;
        m_DriveSubsystem.GetFrontLeftModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetFrontRightModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetBackLeftModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetBackRightModule().setDrivePower(m_vel);
    }

    @Override
    public boolean isFinished(){
        return m_DriveSubsystem.GetFrontLeftModule().getDrivePosition() > 1000;
    }
    
}
