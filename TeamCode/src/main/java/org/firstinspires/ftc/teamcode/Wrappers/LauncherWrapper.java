package org.firstinspires.ftc.teamcode.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Miscellaneous.BulkReadThread;
import org.firstinspires.ftc.teamcode.Miscellaneous.GlobalBulkRead;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.concurrent.atomic.LongAccumulator;

import static java.lang.Thread.sleep;

/**
 *  Wrapper for the ring launcher
 */

@Config
public class LauncherWrapper {
    public ExpansionHubMotor launcherTop;
    public ExpansionHubMotor launcherBottom;
    public ExpansionHubServo launchServo;
    public ExpansionHubServo ringStopper;

    public static double TeleOpShootingVelocity = 600;
    public static double TeleOpPowerShotVelocity = 555;

    public static final double shootingVelocity = 612 - 19.0;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double f = 0;

    public boolean isClosed;
    public boolean isPoweredUp = false;

    public LauncherWrapper(ExpansionHubMotor l_T, ExpansionHubMotor l_B, ExpansionHubServo l_S, ExpansionHubServo ringStopper) {
        this.launcherTop = l_T;
        this.launcherBottom = l_B;
        this.launchServo = l_S;
        this.ringStopper = ringStopper;
    }

    public float getAngularVelocity() {
        if(BulkReadThread.kill) return (float) launcherTop.getVelocity(AngleUnit.DEGREES);

        return GlobalBulkRead.bulkData1.getMotorVelocity(launcherTop);
    }

    public void setVelocity(double angVel, AngleUnit angleUnit) {
        if(angVel == 0.0) { this.stop(); return; }
        isPoweredUp = true;
        launcherTop.setVelocity(angVel,angleUnit);
        launcherBottom.setVelocity(angVel,angleUnit);
    }

    public void setPower(float power) {
        if(power == 0.0) { this.stop(); return; }
        isPoweredUp = true;
        launcherTop.setPower(power);
        launcherBottom.setPower(power);
    }

    public void stop() {
        isPoweredUp = false;
        Hardware.intakeRelease.setPosition(0.33);
        launcherTop.setPower(0);
        launcherBottom.setPower(0);
    }

    public void setPIDFCoeff(PIDFCoefficients newPid) {
        kP = newPid.p; kI = newPid.i; kD = newPid.d; f = newPid.f;
        launcherTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,newPid);
        launcherBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,newPid);
    }

    public PIDFCoefficients getPIDCoeff() {
        return new PIDFCoefficients(kP,kI,kD,f);
    }

    public void setServoPosition(float position) {
        launchServo.setPosition(position);
    }

    public void setStopperPosition(float pos) {
        this.ringStopper.setPosition(pos);
    }

    public void closeStopper() {
        isClosed = true;
        setStopperPosition(0.265f);
    }

    public void openStopper() {
        isClosed = false;
        setStopperPosition(0.11f);
    }

    public void launchOneRing() {
        if(Thread.currentThread().isInterrupted()) return;

        if(isClosed) {
            openStopper();
        }
        setServoPosition(0.5f);
        try {
            sleep(150);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setServoPosition(0.7f);
    }

}
