package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCAutoRedStoneInside extends CCAutoCommon {

    // Constructor
    public CCAutoRedStoneInside()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
        runAuto(true, true);
    }
}