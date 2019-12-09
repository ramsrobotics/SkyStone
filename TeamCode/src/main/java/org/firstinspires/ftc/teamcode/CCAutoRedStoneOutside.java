package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCAutoRedStoneOutside extends CCAutoCommon
{
    // Constructor
    public CCAutoRedStoneOutside()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
        runAuto(false, false);
    }
}
