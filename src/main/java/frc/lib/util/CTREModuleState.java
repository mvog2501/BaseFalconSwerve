package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = desiredState.angle.getDegrees();
    double targetSpeed = desiredState.speedMetersPerSecond;
    if (shouldReverse(Rotation2d.fromDegrees(targetAngle), currentAngle)) {
        targetSpeed = -targetSpeed;
        targetAngle += 180.0;
    }
    targetAngle = placeInAppropriate0to360Scope(currentAngle.getDegrees(), targetAngle);
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  private static boolean shouldReverse(Roation2d targetAngle, Rotation2d currentAngle) {
      double angleDifference = Math.abs(goalAngle.distance(currentAngle));
      double reverseAngleDifference = Math.abs(goalAngle.distance(currentAngle.rotateBy(Rotation2d.fromDegrees(180.0))));
      return reverseAngleDifference < angleDifference;
  }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}
