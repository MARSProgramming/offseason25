package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(
            false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ElevatorIOData(
      boolean masterConnected,
      boolean followerConnected,
      double masterPosition,
      double followerPosition,
      double masterVelocityRadPerSec,
      double followerVelocityRadPerSec,
      double masterAppliedVolts,
      double followerAppliedVolts,
      double masterTorqueCurrentAmps,
      double followerTorqueCurrentAmps,
      double masterSupplyCurrentAmps,
      double followerSupplyCurrentAmps,
      double masterTemp,
      double followerTemp) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void MotionMagic(double position) {}

  default void coast() {}

  default void stop() {}

  default void runPosition(double positionRad) {}

  default void targetLastPosition() {}

  default void setPID(double kP, double kI, double kD) {}

  default void resetPosition() {}

  default void setCharacterization(double kA, double kV, double kG, double kS) {}

  default void setBrakeMode(boolean enabled) {}
}
