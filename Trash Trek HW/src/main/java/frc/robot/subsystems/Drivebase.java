// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//imports CANSparkMax class
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivebase() {
    m_rightMaster.setInverted(true);
  }

  //creates motor objects
  CANSparkMax m_leftMaster  = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_leftSlave   = new CANSparkMax(2, MotorType.kBrushless);

  CANSparkMax m_rightMaster = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_rightSlave  = new CANSparkMax(5, MotorType.kBrushless);


  //timer things
  Timer m_Timer = new Timer();
  public double getTime() {
    return m_Timer.get();
  }

  public void startTimer() {
    m_Timer.start();
  }

  public void resetTimer() {
    m_Timer.reset();
  }

  public void stopTimer() {
    m_Timer.reset();
  }

  
  //creates MotorControllerGroup objects
  MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightSlave);
  MotorControllerGroup m_leftGroup  = new MotorControllerGroup(m_leftMaster,  m_leftSlave);

  //makes an object of the DifferentialDive class
  DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  //makes a method that calls arcadeDrive
  public void arcadeDrive(double speed, double rotation) {
    m_differentialDrive.arcadeDrive(speed, rotation);
  }
  //makes an object of the AHRS class

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}