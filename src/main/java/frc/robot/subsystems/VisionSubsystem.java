// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Represents the vision subsystem.
 *
 * <p>The vision subsystem encapsulates managing cameras attached to the roboRIO, and interfacing
 * with coprocessors to provide object tracking data from the other camera.
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  @Log.CameraStream(
      name = "Camera",
      width = 7,
      height = 6,
      rowIndex = 0,
      columnIndex = 0,
      tabName = "Driver View")
  // Photonvision Camera
  private PhotonCamera m_camera = new PhotonCamera("longwood564");

  /** Initializes the vision subsystem. */
  public VisionSubsystem() {
    
  }

  /** 
   * Gets the latest result from the PhotonVision pipeline. This is called from other methods and should never need to be used on its own. 
   * @return the latest pipeline result.
  */
  private PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  /** 
   * Checks if there are targets on screen. This MUST be called and checked before accessing any of the targets to not possibly throw a NullPointerException.
   * @return whether the camera has detected target(s) or not.
  */
  public boolean hasTargets() {
    return getLatestResult().hasTargets();
  }

  /**
   * Gets the best target from the camera, which should always be the goal, given that we are looking at it.
   * @return the goal, anything else that the camera determines to be a target if the goal is not on screen, or null if there are no targets.
   */
  public PhotonTrackedTarget getTarget() {
    return hasTargets() ? getLatestResult().getBestTarget() : null;
  }

  /**
   * Gets the rotation from the front-center of the robot, where the camera is, to the largest target. Positive values indicate a counter-clockwise rotation, to the left.
   * @return the rotation, in degrees, or null if there is no tracked target.
   */
  public double getRotationToTarget() {
    return hasTargets() ? getTarget().getYaw() : null;
  }

  /**
   * Gets the pitch from the front-center of the robot, where the camera is, to the largest target. Positive values indicate the target is above the camera.
   * @return the pitch, in degrees, or null if there is no tracked target.
   */
  public double getPitchToTarget() {
    return hasTargets() ? getTarget().getPitch() : null;
  }

  /**
   * Gets the calculated area of the largest target, out of 100.
   * @return the area, in percent of total area, or null if there is no tracked target.
   */
  public double getAreaOfTarget() {
    return hasTargets() ? getTarget().getArea() : null;
  }

  /**
   * Gets the calculated distance to the largest target.
   * @return the distance, in meters, or null if there is no tracked target.
   */
  public double getDistanceToTarget() {
    return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kHeightCameraLow, VisionConstants.kHeightTarget, Units.degreesToRadians(VisionConstants.kPitchCameraLow), Units.degreesToRadians(getPitchToTarget()));
  }

}
