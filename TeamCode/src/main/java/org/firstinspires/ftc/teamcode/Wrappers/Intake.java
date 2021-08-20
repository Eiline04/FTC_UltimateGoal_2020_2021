package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

/**
 *  Simple wrapper for the ring intake
 */

public class Intake {

    public ExpansionHubMotor mobileIntake;
    public ExpansionHubMotor staticIntake;
    public CRServo mopStanga;
    public CRServo mopDreapta;
    private boolean reverse;

    public Intake(ExpansionHubMotor sI, ExpansionHubMotor mI, CRServo mopStanga, CRServo mopDreapta) {
        this.mobileIntake = mI;
        this.staticIntake = sI;
        this.mopStanga = mopStanga;
        this.mopDreapta = mopDreapta;
        reverse = false;
    }

    public void releaseIntake() {
        Hardware.intakeRelease.setPosition(0.33);
    }

    public void startIntake() {
        if (reverse) {
            mopStanga.setPower(1);
            mopDreapta.setPower(1);

            mobileIntake.setPower(-1);
            staticIntake.setPower(-1);


        } else {
            mopStanga.setPower(-1);
            mopDreapta.setPower(-1);

            mobileIntake.setPower(1);
            staticIntake.setPower(1);
        }

    }

    public void stopIntake() {
        mobileIntake.setPower(0);
        staticIntake.setPower(0);

        mopStanga.setPower(0);
        mopDreapta.setPower(0);
    }

    public void reverseIntake() {
        reverse = !reverse;
    }


}
