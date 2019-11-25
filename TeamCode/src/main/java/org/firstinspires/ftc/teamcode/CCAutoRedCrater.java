package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCAutoRedCrater extends CCAutoCommon {

    // Constructor
    public CCAutoRedCrater()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
        runAuto(true/*atCrater*/);
    }
}