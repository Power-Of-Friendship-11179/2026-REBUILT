package frc.robot;

import frc.robot.subsystems.TankDrivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;

public class KitBot extends TimedRobot{
	private final XboxController m_controller_drive = new XboxController(0);
	private final TankDrivetrain m_tank = new TankDrivetrain();

	@Override
	public void teleopPeriodic() {
		m_tank.drive(m_controller_drive.getLeftY(), m_controller_drive.getRightY());
	}	
	
}
