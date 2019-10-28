package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCAutoBlueCrater extends CCAutoCommon
{
    // Constructor
    public CCAutoBlueCrater()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(true/*atCrater*/);
    }
}
