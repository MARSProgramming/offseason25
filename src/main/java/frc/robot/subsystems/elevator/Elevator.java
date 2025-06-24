package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private static final LoggedTunableNumber L4Setpoint =
      new LoggedTunableNumber("Elevator/Setpoints/L4");
  private static final LoggedTunableNumber L3Setpoint =
      new LoggedTunableNumber("Elevator/Setpoints/L3");
  private static final LoggedTunableNumber L2Setpoint =
      new LoggedTunableNumber("Elevator/Setpoints/L2");
  private static final LoggedTunableNumber L1Setpoint =
      new LoggedTunableNumber("Elevator/Setpoints/L1");
  private static final LoggedTunableNumber ReefAlgaeTopSetpoint =
      new LoggedTunableNumber("Elevator/Setpoints/ReefTop");
  private static final LoggedTunableNumber ReefAlgaeBotSetpoint =
      new LoggedTunableNumber("Elevator/Setpoints/ReefBot");
  private static final LoggedTunableNumber GroundAlgaeSetpoint =
      new LoggedTunableNumber("Elevator/Setpoints/Ground");
  private static final LoggedTunableNumber ProcessorSetpoint =
      new LoggedTunableNumber("Elevator/Setpoints/Processor");
  private static final LoggedTunableNumber BalanceAlgaeSetpoint =
      new LoggedTunableNumber("Elevator/Setpoints/Balanced");
  private static final LoggedTunableNumber PreclimbSetpoint =
      new LoggedTunableNumber("Elevator/Setpoints/Preclimb");

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/PID/P");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/PID/I");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/PID/D");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/PID/A");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/PID/V");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/PID/S");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/PID/G");

  private static final LoggedTunableNumber ClimbVoltage =
      new LoggedTunableNumber("Elevator/Climbing/ClimbVoltage");
  private static final LoggedTunableNumber ZeroVoltage =
      new LoggedTunableNumber("Elevator/Climbing/ZeroVoltage");
  private static final LoggedTunableNumber AutoZeroVoltage =
      new LoggedTunableNumber("Elevator/Climbing/AutoZeroVoltage");

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final LockIOInputsAutoLogged lockInputs = new LockIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer followerMotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  private final ElevatorIO io;
  private final LockIO lockio;

  public Elevator(ElevatorIO io, LockIO lock) {
    this.io = io;
    this.lockio = lock;
  }

  public void periodic() {
    io.updateInputs(inputs);
    lockio.updateInputs(lockInputs);


    Logger.processInputs("Elevator", inputs);
    Logger.processInputs("ElevatorSafeties", lockInputs);

    motorDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.data.masterConnected()) && !Robot.isJITing());
    followerDisconnectedAlert.set(
        !followerMotorConnectedDebouncer.calculate(inputs.data.followerConnected())
            && !Robot.isJITing());

    // Update tunable numbers (configure before builds and during tuning)
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get());
    }
    //
    //  private static final LoggedTunableNumber LimitDIO = new
    // LoggedTunableNumber("Elevator/Limit/LimitValue");
    //  private static final LoggedTunableNumber ServoPosition = new
    // LoggedTunableNumber("Elevator/Servo/Position");

  }
}
