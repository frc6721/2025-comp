// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;

public class Actuator extends SubsystemBase {

  private final SparkMax m_intakeSparkMax;
  

  private final DigitalInput m_breakBeam1;

  /** Creates a new Actuator. */
  public Actuator(int kintakeid, int kBreakBeam1ID) {
    m_intakeSparkMax = new SparkMax(kintakeid, MotorType.kBrushless);
    m_breakBeam1 = new DigitalInput(ActuatorConstants.kBreakBeam1ID);
  }

  public Boolean hasCorral(){
      return !m_breakBeam1.get();
    
  }

  public Boolean noCorral(){
    return m_breakBeam1.get();

  }

    public void ActuatorIn() {
    m_intakeSparkMax.set(-ActuatorConstants.kActuatorSpeed);
  }

  public void ActuatorAuto() {
    m_intakeSparkMax.set(-ActuatorConstants.kAutoActuatorSpeed);
  }

  public void ActuatorOut() {
    m_intakeSparkMax.set(ActuatorConstants.kActuatorSpeed);
  }

  public Command load(){
    return Commands.startEnd(this::ActuatorIn, this::StopActuator, this).until(this::hasCorral);
  }

  public Command score(){
    return Commands.startEnd(this::ActuatorIn, this::StopActuator, this).until(this::noCorral);
  }

  public Command purge(){
    return Commands.startEnd(this::ActuatorOut, this::StopActuator,this);
  }

  public void StopActuator() {
    m_intakeSparkMax.stopMotor();
  }

  public Command AutoWait(){
    return Commands.waitSeconds(3);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Breakbeam", m_breakBeam1.get());
    // This method will be called once per scheduler run
  }
}
