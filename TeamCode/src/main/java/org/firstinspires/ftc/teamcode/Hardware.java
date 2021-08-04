package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Miscellaneous.BulkReadThread;
import org.firstinspires.ftc.teamcode.Miscellaneous.GlobalBulkRead;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public class Hardware {
    public ExpansionHubMotor frontLeftWheel = null;
    public ExpansionHubMotor frontRightWheel = null;
    public ExpansionHubMotor backLeftWheel = null;
    public ExpansionHubMotor backRightWheel = null;

    public ExpansionHubMotor staticIntake = null;
    public ExpansionHubMotor mobileIntake = null;

    public ExpansionHubMotor launcherTop = null;
    public ExpansionHubMotor launcherBottom = null;

    public ExpansionHubServo launchServo = null;
    public ExpansionHubServo gripperServo = null;
    public ExpansionHubServo armServo = null;
    public ExpansionHubServo ringStopper = null;

    public ExpansionHubEx expansionHub1;
    public ExpansionHubEx expansionHub2;

    public CRServo mopStanga = null;
    public CRServo mopDreapta = null;

    public ExpansionHubServo servoDAS = null;

    public void init(HardwareMap hwMap) {
        GlobalBulkRead.resetBulkData();

        mopStanga = hwMap.get(CRServo.class,"mopStanga");
        mopStanga.setDirection(CRServo.Direction.REVERSE);

        mopDreapta = hwMap.get(CRServo.class, "mopDreapta");
        mopDreapta.setDirection(DcMotorSimple.Direction.FORWARD);

        servoDAS = hwMap.get(ExpansionHubServo.class, "servoDAS");
        //-------------------------------

        expansionHub1 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub2 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        //-----------------------------Wobble Mechanism------------------------------------------
        gripperServo = hwMap.get(ExpansionHubServo.class, "gripperServo");
        armServo = hwMap.get(ExpansionHubServo.class, "armServo");
        armServo.resetDeviceConfigurationForOpMode();

        //-----------------------------Wheels------------------------------------------
        frontLeftWheel = hwMap.get(ExpansionHubMotor.class, "FL");
        frontRightWheel = hwMap.get(ExpansionHubMotor.class, "FR");
        backLeftWheel = hwMap.get(ExpansionHubMotor.class, "BL");
        backRightWheel = hwMap.get(ExpansionHubMotor.class, "BR");

        //-----------------------------Intake------------------------------------------
        staticIntake = hwMap.get(ExpansionHubMotor.class, "staticIntake");
        mobileIntake = hwMap.get(ExpansionHubMotor.class, "mobileIntake");

        //-----------------------------Launcher------------------------------------------
        launcherTop = hwMap.get(ExpansionHubMotor.class, "launcherTop");
        launcherBottom = hwMap.get(ExpansionHubMotor.class, "launcherBottom");
        launchServo = hwMap.get(ExpansionHubServo.class, "launcherServo");
        ringStopper = hwMap.get(ExpansionHubServo.class, "ringStopper");

        frontLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        frontRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);
        backLeftWheel.setDirection(ExpansionHubMotor.Direction.FORWARD);
        backRightWheel.setDirection(ExpansionHubMotor.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);

        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);

        staticIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        staticIntake.setDirection(DcMotorEx.Direction.FORWARD);
        staticIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        mobileIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mobileIntake.setDirection(DcMotorEx.Direction.REVERSE);
        mobileIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        launcherTop.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherBottom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherTop.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcherBottom.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        launcherTop.setDirection(DcMotorEx.Direction.FORWARD);
        launcherBottom.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void enableBulkDataPolling() {
        GlobalBulkRead.resetBulkData();
        GlobalBulkRead.setExpansionHubs(expansionHub1, expansionHub2);
        BulkReadThread bulkReadThread = new BulkReadThread();
        Thread bulkRunner = new Thread(bulkReadThread);
        bulkRunner.start();
    }

}
