/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
 
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Logger;
import robotCore.RobotCoreBase;
 
/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDrive extends Command {
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_turn;
  private final Boolean m_fieldRelative;
 
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
 
  /**
   * Creates a new ArcadeDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier turn,
      boolean fieldRelative) {
    Logger.log("ArcadeDrive", 3, "ArcadeDrive()");
 
    m_subsystem = subsystem;
    m_x = x;
    m_y = y;
    m_turn = turn;
    m_fieldRelative = fieldRelative;
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDrive", 2, "initialize()");
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("ArcadeDrive", -1, "execute()");
 
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double turn = m_turn.getAsDouble();
 
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(y, 0.02))
        * Constants.Drive.k_maxDriveSpeedMetersPerSecond;
 
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(x, 0.02))
        * Constants.Drive.k_maxDriveSpeedMetersPerSecond;
 
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(turn, 0.20))
        * Constants.Drive.k_maxSpeed;
 
    if ((xSpeed != 0) || (ySpeed != 0) || (rot != 0)) {
      m_subsystem.drive(xSpeed, ySpeed, rot, m_fieldRelative, ((TimedRobot) RobotCoreBase.getInstance()).getPeriod());
    } else {
      m_subsystem.stop();
    }
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ArcadeDrive", 2, String.format("end(%b)", interrupted));
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("ArcadeDrive", -1, "isFinished()");
    return false;
  }
}