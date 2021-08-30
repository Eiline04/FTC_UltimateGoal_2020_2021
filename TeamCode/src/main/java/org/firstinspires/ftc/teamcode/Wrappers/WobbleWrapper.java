package org.firstinspires.ftc.teamcode.Wrappers;

import org.openftc.revextensions2.ExpansionHubServo;

public class WobbleWrapper {
    public ExpansionHubServo gripper;
    public ExpansionHubServo arm;
    public ExpansionHubServo wobbleRelease;

    public WobbleWrapper(ExpansionHubServo gripper, ExpansionHubServo arm, ExpansionHubServo wobbleRelease) {
        this.gripper = gripper;
        this.arm = arm;
        this.wobbleRelease = wobbleRelease;
    }

    public void wobbleRelease() {
        wobbleRelease.setPosition(0.5);
    }

    public void resetWobbleRelease() {
        wobbleRelease.setPosition(0.85);
    }

    public void attachGrip() {
        gripper.setPosition(0.30);
    }

    public void detachGrip() {
        gripper.setPosition(0.9);
    }

    public void setArmPosition(float position) {
        arm.setPosition(position);
    }

    public void closeArm() {
        arm.setPosition(0.015);
    }

    public void openArm() {
        arm.setPosition(1.0);
    }

}
