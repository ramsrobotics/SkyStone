package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCAutoBlueStoneOutside extends CCAutoCommon
{
    // Constructor
    public CCAutoBlueStoneOutside()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        runAuto(false, true);
    }
}
