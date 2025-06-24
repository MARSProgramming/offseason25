package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class LockIOServoAndMagswitch implements LockIO {

    private final DigitalInput limit;
    private final Servo servo;

    private final double servoPosition;
    private final double servoAngle;
    private final boolean limitValue;

    public LockIOServoAndMagswitch() {
        limit = new DigitalInput(0); // Adjust the channel as needed
        servo = new Servo(1); // Adjust the channel as needed
        servoPosition = servo.getPosition(); // Default servo position
        servoAngle = servo.getAngle(); // Default servo position
        limitValue = limit.get(); // Default limit value
    }

    @Override
    public void update(LockIOInputs inputs) {
        inputs.data = new LockIOData(
            limit.get(),
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
