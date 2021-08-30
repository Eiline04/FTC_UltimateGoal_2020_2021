package org.firstinspires.ftc.teamcode.Wrappers;

import org.openftc.revextensions2.ExpansionHubServo;

public class DasPositions {

    ExpansionHubServo servoDAS;

    public DasPositions(ExpansionHubServo servoDAS) {
        this.servoDAS = servoDAS;
    }

    public void setPositionDAS(double positionDAS) {
        servoDAS.setPosition(positionDAS);
    }

    public void startDAS() {
        setPositionDAS(1.00);
    }
}
