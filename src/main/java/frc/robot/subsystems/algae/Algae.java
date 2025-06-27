package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  // TODO: add default values

  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  private final AlgaeIO io;
  double threshold;

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  public Command run(double percentOut, boolean passiveActive) {
    return runEnd(
        () -> {
          io.setOutput(percentOut);
        },
        () -> {
          if (passiveActive) {
            io.setOutput(0.5);
          } else {
            io.setOutput(0);
          }
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Algae", inputs);
  }
}
