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
  public Elevator() {}

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
  CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_motor.getEncoder(Type.kQuadrature, 0);

  public void resetEncoders() {
    m_encoder.setPosition(0);
  }
  public double getPos() {
    return m_encoder.getPosition();
  }

  public SparkMaxLimitSwitch getLimitSwitch() {
    return m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }
  private double power;

  public void setVolt(double volts) {
    power = volts;
    m_motor.setVoltage(power);
  }

  public void softLimit(double amount) {
    // Sets the soft limit to amount
    m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) amount);
  }

  public void smartDash() {
    SmartDashboard.putBoolean("limitSwitchPressed", getLimitSwitch().isPressed());
    SmartDashboard.putNumber("encoderHeight", m_encoder.getPosition());
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
