// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Drives the robot towards the closest powercell for use in the Galactic Search challenge. */
public class DriveToPowercellCommand extends CommandBase {

  // State variables
  public static final int m_STATE_SEARCHING = 0;
  public static final int m_STATE_TURNING = 1;
  public static final int m_STATE_DRIVING_TOWARDS_POWERCELL = 2;
  public static final int m_STATE_TURN_180 = 3;
  public static final int m_STATE_GRABBING_POWERCELL = 4;
  public static final int m_STATE_TURNING_TOWARDS_END = 5;
  public static final int m_STATE_DRIVING_TOWARDS_END = 6;
  public static final int m_STATE_FINISHED = 7;

  // Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final VisionSubsystem m_visionSubsystem;

  // State of the command
  int m_state;

  // Ball count
  int m_ballCount;

  // Initial time for intaking the powercell
  long m_intakeInitTime;

  // Initial angle for when turning 180 degrees
  double m_initAngle;

  // Initial angle for when the command is executed
  double m_absoluteInitAngle;

  /** Initializes this command. */
  public DriveToPowercellCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem,
      VisionSubsystem visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_visionSubsystem = visionSubsystem;

    addRequirements(drivetrainSubsystem);
    addRequirements(m_intakeSubsystem);
    addRequirements(m_visionSubsystem);
  }

  /** This method is run once when the command is first executed. */
  @Override
  public void initialize() {
    m_state = m_STATE_SEARCHING;
    m_ballCount = 0;
    m_initAngle = 0;
    m_absoluteInitAngle = m_drivetrainSubsystem.getHeading();
  }

  /** This method is run continously while the command is executed. */
  @Override
  public void execute() {
    // Set the neutral mode of the drive motors to break, for more responsiveness.
    m_drivetrainSubsystem.setNeutralMode(NeutralMode.Brake);

    if (m_state == m_STATE_DRIVING_TOWARDS_END) {
      Pose2d position = m_drivetrainSubsystem.getPose();
      double distanceToEdge = Units.feetToMeters(30) - position.getX();
      if (distanceToEdge >= Units.feetToMeters(2.5)) {
        m_drivetrainSubsystem.arcadeDrive(
            m_visionSubsystem.getNonlinearSpeed(distanceToEdge, Units.feetToMeters(2.5), 0.5), 0);
      }
      m_state = m_STATE_DRIVING_TOWARDS_END;
      return;
    } else {
      m_state = m_STATE_FINISHED;
    }
  }

  /** This method is run when the command is done. */
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.arcadeDrive(0, 0);
  }

  /**
   * Gets the state of the command.
   *
   * @return the state of the command
   * @see DriveToPowercellCommand.m_STATE_SEARCHING
   * @see DriveToPowercellCommand.m_STATE_TURNING
   * @see DriveToPowercellCommand.m_STATE_DRIVING_TOWARDS_POWERCELL
   * @see DriveToPowercellCommand.m_STATE_TURN_180
   * @see DriveToPowercellCommand.m_STATE_GRABBING_POWERCELL
   * @see DriveToPowercellCommand.m_STATE_DRIVING_TOWARDS_END
   * @see DriveToPowercellCommand.m_STATE_FINISHED
   */
  public int getState() {
    return m_state;
  }

  /** This method is run to check whether the command is finished. */
  @Override
  public boolean isFinished() {
    return m_state == m_STATE_FINISHED;
  }
}
