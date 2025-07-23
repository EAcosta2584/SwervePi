package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;

public class DriveTuning extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private Timer m_timer = new Timer();

    private double m_vel;

    public DriveTuning(DriveSubsystem subsystem){
        m_DriveSubsystem = subsystem;
        
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize(){
        m_vel = 0;
        //m_DriveSubsystem.GetFrontLeftModule().setSteeringPosition(0);
        //m_DriveSubsystem.GetFrontRightModule().setSteeringPosition(0);
        //m_DriveSubsystem.GetBackLeftModule().setSteeringPosition(0);
        //m_DriveSubsystem.GetBackRightModule().setSteeringPosition(0);
        
    }

    @Override
    public void execute(){
        m_timer.reset();
        m_vel = m_vel + 0.05;
        m_DriveSubsystem.GetFrontLeftModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetFrontRightModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetBackLeftModule().setDrivePower(m_vel);
        m_DriveSubsystem.GetBackRightModule().setDrivePower(m_vel);
        Logger.log("FL velocity:", 3, String.valueOf(m_vel));
        m_timer.start();
        while (m_timer.get() < 2){
            Logger.log("pause", -1, "wait");
        }
    }

    @Override
    public void end(boolean interrupted){
        m_vel = 0;
        m_DriveSubsystem.GetFrontLeftModule().setSteeringPower(m_vel);
        m_DriveSubsystem.GetFrontRightModule().setSteeringPower(m_vel);
        m_DriveSubsystem.GetBackLeftModule().setSteeringPower(m_vel);
        m_DriveSubsystem.GetBackRightModule().setSteeringPower(m_vel);
    }

    @Override
    public boolean isFinished(){
        return(m_vel >= 1.2);
    }
}
