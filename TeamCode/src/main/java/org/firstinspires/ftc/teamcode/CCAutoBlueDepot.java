package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCAutoBlueDepot extends CCAutoCommon
{
    // Constructor
    public CCAutoBlueDepot()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        runAuto(false/*atCrater*/);
    }
}
