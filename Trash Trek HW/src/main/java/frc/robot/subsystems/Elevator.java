// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    m_encoder.setPosition(0);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_motor.getEncoder(Type.kQuadrature, 0);

  public PIDController m_PIDController = new PIDController(0.1, 0.1, 0.1);

  SparkMaxLimitSwitch m_LimitSwitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  
  public void resetEncoders() {
    m_encoder.setPosition(0);
  }

  public double getEncoderPositon() {
    return m_encoder.getPosition();
  }

  public boolean getLimitSwitchEnabled() {
    return m_LimitSwitch.isLimitSwitchEnabled();
  }

  public double setHeightWithConstraint(double height){
    double output = m_PIDController.calculate(m_encoder.getPosition(), height);
    return output;
  }

  public void displayElevatorHeight() {
    SmartDashboard.putNumber("elevatorHeight", m_encoder.getPosition());
  }
   
  
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("limitSwitchPressed", getLimitSwitchEnabled());
    displayElevatorHeight();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
