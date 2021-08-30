package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.openftc.revextensions2.ExpansionHubMotor;

/**
 * Simple wrapper for the ring intake and auxiliary mechanisms
 */

public class Intake {

    public ExpansionHubMotor mobileIntake;
    public ExpansionHubMotor staticIntake;
    public CRServo leftOmni;
    public CRServo rightOmni;
    private boolean reverse;

    public Intake(ExpansionHubMotor sI, ExpansionHubMotor mI, CRServo leftOmni, CRServo rightOmni) {
        this.mobileIntake = mI;
        this.staticIntake = sI;
        this.leftOmni = leftOmni;
        this.rightOmni = rightOmni;
        reverse = false;
    }

    public void releaseIntake() {
        Hardware.intakeRelease.setPosition(0.33);
    }

    public void startIntake() {
        if (reverse) {
            leftOmni.setPower(1);
            rightOmni.setPower(1);

            mobileIntake.setPower(-1);
            staticIntake.setPower(-1);


        } else {
            leftOmni.setPower(-1);
            rightOmni.setPower(-1);

            mobileIntake.setPower(1);
            staticIntake.setPower(1);
        }

    }

    public void stopIntake() {
        mobileIntake.setPower(0);
        staticIntake.setPower(0);

        leftOmni.setPower(0);
        rightOmni.setPower(0);
    }

    public void reverseIntake() {
        reverse = !reverse;
    }


}
