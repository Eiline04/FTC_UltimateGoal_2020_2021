package org.firstinspires.ftc.teamcode.Roadrunner.util;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Miscellaneous.BulkReadThread;
import org.firstinspires.ftc.teamcode.Miscellaneous.GlobalBulkRead;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction
 */
public class Encoder {
    private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;
    private NanoClock clock;

    private Direction direction;

    private int lastPosition;
    private double velocityEstimate;
    private double lastUpdateTime;

    public Encoder(DcMotorEx motor, NanoClock clock) {
        this.motor = motor;
        this.clock = clock;

        this.direction = Direction.FORWARD;

        this.lastPosition = 0;
        this.velocityEstimate = 0.0;
        this.lastUpdateTime = clock.seconds();
    }

    public Encoder(DcMotorEx motor) {
        this(motor, NanoClock.system());
    }

    public Direction getDirection() {
        return direction;
    }

    private int getMultiplier() {
        return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     *
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * Similar to getCurrentPosition(), except it polls straight from the Bulk Data
     *
     * @return current position of the encoder in ticks
     */
    public int getCurrentPositionBulk() {
        int multiplier = getMultiplier();

        //If thread is inactive it will cause an error. Just use normal method.
        if (BulkReadThread.kill) return getCurrentPosition();

        int currentPosition = GlobalBulkRead.bulkData2.getMotorCurrentPosition(motor) * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public double getRawVelocity() {
        int multiplier = getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    /**
     * Similar to getRawVelocity(), except it polls straight from the Bulk Data
     *
     * @return current velocity of the encoder in ticks/sec
     */
    public double getRawVelocityBulk() {
        //If thread is inactive it will cause an error. Just use normal method.
        if (BulkReadThread.kill) return getRawVelocity();

        int multiplier = getMultiplier();
        return GlobalBulkRead.bulkData2.getMotorVelocity(motor) * multiplier;
    }

    public double getCorrectedVelocity() {
        return inverseOverflow(getRawVelocityBulk(), velocityEstimate);
    }
}
