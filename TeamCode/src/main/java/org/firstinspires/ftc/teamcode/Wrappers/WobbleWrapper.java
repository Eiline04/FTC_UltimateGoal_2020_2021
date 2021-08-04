package org.firstinspires.ftc.teamcode.Wrappers;

import org.openftc.revextensions2.ExpansionHubServo;

public class WobbleWrapper {
    public ExpansionHubServo gripper;
    public ExpansionHubServo arm;

    public WobbleWrapper(ExpansionHubServo gripper, ExpansionHubServo arm) {
        this.gripper = gripper;
        this.arm = arm;
    }

    public void attachGrip() {
        gripper.setPosition(0.40);
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
        arm.setPosition(0.9);
    }

}
