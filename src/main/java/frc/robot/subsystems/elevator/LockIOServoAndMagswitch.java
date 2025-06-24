package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class LockIOServoAndMagswitch implements LockIO {

    private final DigitalInput limit;
    private final Servo servo;


    public LockIOServoAndMagswitch() {
        limit = new DigitalInput(0); // Adjust the channel as needed
        servo = new Servo(1); // Adjust the channel as needed
    }

    @Override
    public void updateInputs(LockIOInputs inputs) {
        inputs.data = new LockIOData(
            !limit.get(), // assume normal low
            servo.get(),
            servo.getAngle()
            );
    }

    @Override
    public void setServoAngle(double angleDeg) {
        servo.setAngle(angleDeg);
    }

    @Override
    public void setServo(double position) {
        servo.set(position);
    }

}
