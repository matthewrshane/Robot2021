// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Drives the robot towards the closest powercell for use in the Galactic Search challenge. */
public class Turn180Command extends CommandBase {
  // Subsystems

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final VisionSubsystem m_visionSubsystem;

  // Boolean
  boolean m_finished = false;

  // Initial angle for when turning 180 degrees
  double m_initAngle = 0;

  /** Initializes this command. */
  public Turn180Command(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run when the command is executed. */
  @Override
  public void execute() {
    // Set the neutral mode of the drive motors to break, for more responsiveness.
    m_drivetrainSubsystem.setNeutralMode(NeutralMode.Brake);

    // Turn 180 degrees
    double angleRemaining = m_drivetrainSubsystem.getGyroHeading() - (m_initAngle + 180);
    if (Math.abs(angleRemaining) >= VisionConstants.kRotationTolerance) {
      m_drivetrainSubsystem.arcadeDrive(
          0, m_visionSubsystem.getNonlinearRotationalSpeed(angleRemaining));
      return;
    }

    m_finished = true;
  }

  /** This method is run when the command is done. */
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.arcadeDrive(0, 0);
  }

  /** This method is run to check whether the command is finished. */
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
