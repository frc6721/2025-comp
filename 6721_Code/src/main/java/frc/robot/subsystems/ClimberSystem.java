// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ClimberSystem extends SubsystemBase {

  private final SparkMax m_climbMotor = new SparkMax(ClimberConstants.kClimberID, MotorType.kBrushless);
  private final RelativeEncoder m_climbEncoder = m_climbMotor.getEncoder(); 

  private final DigitalInput m_photoEye;

  private double climberCurrentTarget = ClimberConstants.kClimbPos;
  private SparkClosedLoopController climbClosedLoopController = m_climbMotor.getClosedLoopController();

  public enum climbSetpoint {
    kStow,
    kClimbPos,
  }

  /** Creates a new ClimberSystem. */
  public ClimberSystem(int photoeyeDIO) {
    m_photoEye = new DigitalInput(photoeyeDIO);

    m_climbMotor.configure(Configs.ClimberSubsystem.climbConfig, 
    ResetMode.kResetSafeParameters, 
    PersistMode.kPersistParameters);
  }

  public void ClimbUp() {
    m_climbMotor.set(ClimberConstants.kClimbSpeed);
  }
  public void ClimbDown() {
    m_climbMotor.set(-ClimberConstants.kClimbSpeed);
  }

  public boolean GetPhotoeye() {
    return m_photoEye.get();
  }

  public void StopClimb() {
    m_climbMotor.stopMotor();
  }

  public Command Climb(){
    return Commands.startEnd(this::ClimbUp, this::StopClimb, this);
  }

  public Command Down(){
    return Commands.startEnd(this::ClimbDown, this::StopClimb,this);
  }

  private void moveToSetpoint(){
    climbClosedLoopController.setReference(
      climberCurrentTarget,
      ControlType.kMAXMotionPositionControl
    );
  }

   public Command setSetpointCommand(climbSetpoint setpoint){
      return this.runOnce(
        () -> {
          switch (setpoint){
            case kStow:
            climberCurrentTarget = ClimberConstants.kStow;
            break;
            case kClimbPos:
            climberCurrentTarget = ClimberConstants.kClimbPos;
            break;
          }
        }
      );
    } 

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //moveToSetpoint();
    
    SmartDashboard.putNumber("Elevator Target Position" , climberCurrentTarget);
    SmartDashboard.putNumber("Elevator Actual Position" , m_climbEncoder.getPosition());
  }
}
