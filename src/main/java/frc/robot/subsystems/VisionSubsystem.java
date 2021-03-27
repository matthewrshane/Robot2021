// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

/**
 * Represents the vision subsystem.
 *
 * <p>The vision subsystem encapsulates managing cameras attached to the roboRIO, and interfacing
 * with coprocessors to provide object tracking data from the other camera.
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  // Photonvision Camera
  private PhotonCamera m_camera = new PhotonCamera("longwood564");

  /** Initializes the vision subsystem. */
  public VisionSubsystem() {}

  /**
   * Gets the latest result from the PhotonVision pipeline. This is called from other methods and
   * should never need to be used on its own.
   *
   * @return the latest pipeline result.
   */
  private PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  /**
   * Checks if there are targets on screen. This MUST be called and checked before accessing any of
   * the targets to not possibly throw a NullPointerException.
   *
   * @return whether the camera has detected target(s) or not.
   */
  public boolean hasTargets() {
    return getLatestResult().hasTargets();
  }

  /**
   * Gets the best (largest) target from the camera, given that we are looking at it.
   *
   * @return the goal, anything else that the camera determines to be a target if the goal is not on
   *     screen, or null if there are no targets.
   */
  public PhotonTrackedTarget getTarget() {
    return hasTargets() ? getLatestResult().getBestTarget() : null;
  }

  /**
   * Gets the rotation from the front-center of the robot, where the camera is, to the largest
   * target. Positive values indicate a counter-clockwise rotation, to the left.
   *
   * @return the rotation, in degrees, or null if there is no tracked target.
   */
  public double getRotationToTarget() {
    return hasTargets() ? getTarget().getYaw() : null;
  }

  /**
   * Gets the pitch from the front-center of the robot, where the camera is, to the largest target.
   * Positive values indicate the target is above the camera.
   *
   * @return the pitch, in degrees, or null if there is no tracked target.
   */
  public double getPitchToTarget() {
    return hasTargets() ? getTarget().getPitch() : null;
  }

  /**
   * Gets the calculated area of the largest target, out of 100.
   *
   * @return the area, in percent of total area, or null if there is no tracked target.
   */
  public double getAreaOfTarget() {
    return hasTargets() ? getTarget().getArea() : null;
  }

  /**
   * Gets the calculated distance to the largest target.
   *
   * @return the distance, in meters, or null if there is no tracked target.
   */
  public double getDistanceToTarget() {
    return hasTargets()
        ? (7 * 700.8) / Math.sqrt(getLatestResult().getBestTarget().getArea() * 320 * 240 * 0.01)
        : null;
    // return PhotonUtils.calculateDistanceToTargetMeters(
    //     VisionConstants.kHeightCameraLow,
    //     VisionConstants.kHeightPowercell,
    //     Units.degreesToRadians(VisionConstants.kPitchCameraLow),
    //     Units.degreesToRadians(getPitchToTarget()));
  }

  /**
   * Gets a non-linear curve for speed based off the current distance and the minimum distance we
   * would like to be from the target.
   *
   * @param distance The current distance from the target, in meters.
   * @param minimumDistance The minimum distance the robot should approach the target, in meters.
   * @param maxSpeed The maximum speed that the robot can drive at. Must be between 0.0 and 1.0, or
   *     it will be cut off.
   * @return the speed, from 0.0 to 1.0, that the robot should travel at.
   */
  public double getNonlinearSpeed(double distance, double minimumDistance, double maxSpeed) {
    return (maxSpeed) * (1D / (1 + Math.exp(-(distance - minimumDistance - 2.25) / 0.75)));
  }

  /**
   * Gets a non-linear curve for rotational speed based on the current angle offset we are to where
   * we would like to be facing.
   *
   * @param rotation The current offset, in degrees, from the angle we would like to be facing.
   * @param maxRotationSpeed The maximum rotation speed that the robot can turn at. Must be between
   *     0.0 and 1.0, or it will be cut off.
   * @return the rotational speed, from -maxRotationSpeed to +maxRotationSpeed, that the robot
   *     should turn at.
   */
  public double getNonlinearRotationalSpeed(double rotation, double maxRotationSpeed) {
    boolean sign = rotation >= 0;
    rotation = Math.abs(rotation);
    return (sign ? 1 : -1) * (maxRotationSpeed) * (1D / (1 + Math.exp(-(rotation - 90) / 30)));
  }
}
