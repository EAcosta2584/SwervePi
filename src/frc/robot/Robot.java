package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import robotCore.Logger;

public class Robot extends TimedRobot {
	@SuppressWarnings("unused")
	private RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		Logger.log("Robot", 3, "robotInit()");
		m_robotContainer = new RobotContainer();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		Logger.log("Robot", -1, "robotPeriodic()");
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		commandScheduler();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
		Logger.log("Robot", 2, "disabledInit()");
	}

	@Override
	public void disabledPeriodic() {
		Logger.log("Robot", -1, "disbledPeriodic()");
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		Logger.log("Robot", 2, "autonomousInit()");
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Logger.log("Robot", -1, "autonomousPeriodic()");
	}

	@Override
	public void teleopInit() {
		Logger.log("Robot", 2, "teleopInit()");
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Logger.log("Robot", -1, "teleopPeriodic()");
	}

	@Override
	public void testInit() {
		Logger.log("Robot", 2, "testInit()");
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		Logger.log("Robot", -1, "testPeriodic()");
	}
}
