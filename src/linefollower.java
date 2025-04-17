package linefollower;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {

        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];

        int baseSpeed = 255;
        int maxSpeed = 600;
        int minSpeed = 100;

        // Create PID controller with custom parameters
        PIDController pid = new PIDController(300, 50, 200, 0.2f);

        LCD.drawString("PID with Class", 0, 0);
        Delay.msDelay(1000);

        while (!Button.ESCAPE.isDown()) {
            light.fetchSample(sample, 0);
            float lightValue = sample[0];

            float correction = pid.calculate(lightValue);

            int leftSpeed = (int)(baseSpeed + correction);
            int rightSpeed = (int)(baseSpeed - correction);

            leftSpeed = Math.max(minSpeed, Math.min(maxSpeed, leftSpeed));
            rightSpeed = Math.max(minSpeed, Math.min(maxSpeed, rightSpeed));

            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            LCD.clear();
            LCD.drawString("Light: " + (int)(lightValue * 100) + "%", 0, 1);
            LCD.drawString("L:" + leftSpeed + " R:" + rightSpeed, 0, 2);

            Delay.msDelay(40);
        }

        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
