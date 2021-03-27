// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GrabPowercellCommand extends CommandBase {

  // Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  // Is the command finished.
  private boolean m_finished;

  // Is the command functioning properly.
  private boolean m_functioning;

  // Initial time for intaking the powercell
  long m_intakeInitTime;

  /** Initializes this command. */
  public GrabPowercellCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      VisionSubsystem visionSubsystem,
      IntakeSubsystem intakeSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(m_drivetrainSubsystem);
    addRequirements(m_visionSubsystem);
    addRequirements(m_intakeSubsystem);
  }

  /** This method is run once when the command is first executed. */
  @Override
  public void initialize() {
    m_finished = false;
    m_functioning = true;

    m_intakeInitTime = System.currentTimeMillis();
  }

  /** This method is run continously while the command is executed. */
  @Override
  public void execute() {
    if (!m_visionSubsystem.hasTargets()) {
      m_functioning = false;
      return;
    }

    if (System.currentTimeMillis() - m_intakeInitTime <= 3000) {
      m_intakeSubsystem.startIntake(Constants.IntakeConstants.kSpeedIntake);
      m_intakeSubsystem.startBelt(Constants.IntakeConstants.kSpeedBelt);
      m_drivetrainSubsystem.arcadeDrive(-0.3, 0);
    } else {
      m_intakeSubsystem.startIntake(0);
      m_intakeSubsystem.startBelt(0);
      m_drivetrainSubsystem.arcadeDrive(0, 0);
    }
    m_finished = true;
  }

  /** This method is run to check whether the command is finished. */
  @Override
  public boolean isFinished() {
    return m_finished;
  }

  /**
   * This method is run to check whether the command is functioning properly. If not, the previous
   * command must be reverted to.
   */
  public boolean isFunctional() {
    return m_functioning;
  }
}
