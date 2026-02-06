package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class TankDrivetrain {
	private static final double kWheelRadius = 0.0762;
  private static final int kEncoderResolution = 4096;

	private final DifferentialDrive m_drive;

	private final PWMSparkMax m_leftMotor;
	private final PWMSparkMax m_rightMotor;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

	public TankDrivetrain() {
		m_leftMotor = new PWMSparkMax(1);
		m_rightMotor = new PWMSparkMax(2);
		m_rightMotor.setInverted(true);
		m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

		m_leftEncoder = new Encoder(0, 1);
		m_rightEncoder = new Encoder(2, 3);

		m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
		m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

		SendableRegistry.addChild(m_drive, m_leftMotor);
		SendableRegistry.addChild(m_drive, m_rightMotor);
	}

	public void drive(double left_speed, double right_speed) {
		m_drive.tankDrive(left_speed, right_speed);
	}
}