// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Configs.ClimberSubsystem;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Actuator;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSystem.climbSetpoint;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static edu.wpi.first.units.Units.derive;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Actuator m_Actuator = new Actuator(ActuatorConstants.kIntakeID, ActuatorConstants.kBreakBeam1ID);
  public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ClimberSystem m_ClimberSubsystem = new ClimberSystem(ClimberConstants.kPhotoID);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller

    m_driverController.leftBumper().whileTrue(new RunCommand(
      () -> m_robotDrive.setX(), 
      m_robotDrive));
    
    m_driverController.rightBumper().whileTrue(m_Actuator.purge());

    m_driverController.rightTrigger().whileTrue(m_Actuator.score());
            
    m_driverController.leftTrigger().whileTrue(m_Actuator.load());

    m_driverController.a().whileTrue(m_ElevatorSubsystem.autoMoveElevatorL4());

    m_driverController.x().whileTrue(m_ClimberSubsystem.Down());

    m_driverController.y().whileTrue(m_ClimberSubsystem.Climb());

    

    // Operator Controller

    m_operatorController.rightTrigger().whileTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kL1));

    m_operatorController.b().whileTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kL2));

    m_operatorController.a().whileTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kL3));

    m_operatorController.leftTrigger().whileTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kL4));

    m_operatorController.rightBumper().whileTrue(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kStow));
            
    m_operatorController.x().whileTrue(m_ClimberSubsystem.setSetpointCommand(climbSetpoint.kClimbPos));

    m_operatorController.leftBumper().whileTrue(m_ClimberSubsystem.setSetpointCommand(climbSetpoint.kStow));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making straight line
        List.of(new Translation2d(.25, 0 ), new Translation2d(1, 0)),
        // End 1.5 meters straight ahead of where we started, facing forward
        new Pose2d(1.4, 0, new Rotation2d(-0.098)),
        config);
    Trajectory leftTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making straight line
          List.of(new Translation2d(.25, .75 ), new Translation2d(1, 1.5)),
          // End 1.5 meters straight ahead of where we started, facing forward
          new Pose2d(2, 2.75, new Rotation2d(45)),
          config);  
    Trajectory rightTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making straight line
            List.of(new Translation2d(.25, -.75 ), new Translation2d(1, -1.5)),
            // End 1.5 meters straight ahead of where we started, facing forward
            new Pose2d(2, -2.75, new Rotation2d(-45)),
            config);            

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        straightTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    
    return swerveControllerCommand.andThen(m_ElevatorSubsystem.setSetpointCommand(Setpoint.kL4))
                                  .andThen(() -> m_robotDrive.drive(0, 0, 0, false))
                                  .andThen(m_Actuator.AutoWait())
                                  .andThen(() -> m_Actuator.ActuatorAuto())
                                  .andThen(m_Actuator.AutoWait())
                                  .andThen(() -> m_Actuator.StopActuator());
                                  
                                  
                              
  }
}
