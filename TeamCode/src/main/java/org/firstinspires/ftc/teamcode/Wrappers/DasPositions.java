package org.firstinspires.ftc.teamcode.Wrappers;

import org.openftc.revextensions2.ExpansionHubServo;

public class DasPositions {

    ExpansionHubServo servoDAS;
    public double changePos = 0.002;

    public double currentDASPosition;

    public DasPositions(ExpansionHubServo servoDAS){
        this.servoDAS = servoDAS;
    }

    /*1 = left
    * 2 = middle
    * 3 = right
    * 0 = start position*/

    public void setPositionDAS(double positionDAS){servoDAS.setPosition(positionDAS);}

    public void startDAS(){ setPositionDAS(1.00);}

    public void leftDAS(){setPositionDAS(0.685);} // was 0.725

    public void middleDAS(){setPositionDAS(0.645);}   //0.685

    public void rightDAS(){setPositionDAS(0.589);} //best 0.589// after 0.575 // 0.645

    public void toRightDAS(){
        currentDASPosition = servoDAS.getPosition();
        if(currentDASPosition > 0.5){
            servoDAS.setPosition(currentDASPosition - changePos);
        }
    }

    public void toLeftDAS(){
        currentDASPosition = servoDAS.getPosition();
        if(currentDASPosition < 0.9){
            servoDAS.setPosition(currentDASPosition + changePos);
        }
    }
}
