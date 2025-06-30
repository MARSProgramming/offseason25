package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public class AlgaeIOInputs {
    public AlgaeIOData data = new AlgaeIOData(0.0, 0.0, 0.0, 0.0);
  }

  record AlgaeIOData(double percOut, double current, double voltage, double temperature) {}

  default void updateInputs(AlgaeIOInputs inputs) {}

  default void setOutput(double percent) {}
}
