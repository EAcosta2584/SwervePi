package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import robotCore.Logger;
import frc.robot.Constants.Steering;

public class SteeringToAngle extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final SwerveModule m_FLmodule;
    private final SwerveModule m_BLmodule;
    private final SwerveModule m_BRmodule;
    private final SwerveModule m_FRmodule;
    private SwerveModule m_module;
    private final double m_angle;

    public SteeringToAngle(DriveSubsystem subsystem, double angle){
        m_DriveSubsystem = subsystem;
        m_angle = angle;
        m_FLmodule = m_DriveSubsystem.GetFrontLeftModule();
        m_BLmodule = m_DriveSubsystem.GetBackLeftModule();
        m_BRmodule = m_DriveSubsystem.GetBackRightModule();
        m_FRmodule = m_DriveSubsystem.GetFrontRightModule();
        
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize(){

        m_FLmodule.setSteeringPosition(m_angle);
        m_FRmodule.setSteeringPosition(m_angle);
        m_BLmodule.setSteeringPosition(m_angle);
        m_BRmodule.setSteeringPosition(m_angle);

    }

    @Override
    public void execute(){

        Logger.log("SteerAngle", 1, String.valueOf(m_FLmodule.getSteeringPositionInDegrees()));
    }

    @Override
    public void end(boolean interrupted){
        m_DriveSubsystem.GetFrontLeftModule().setSteeringPower(0);
        m_DriveSubsystem.GetFrontRightModule().setSteeringPower(0);
        m_DriveSubsystem.GetBackLeftModule().setSteeringPower(0);
        m_DriveSubsystem.GetBackRightModule().setSteeringPower(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
