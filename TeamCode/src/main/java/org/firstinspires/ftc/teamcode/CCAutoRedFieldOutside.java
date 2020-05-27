package org.firstinspires.ftc.teamcode;

public class CCAutoRedFieldOutside extends CCAutoCommon {
    // Constructor
    public CCAutoRedFieldOutside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() {
        runAuto(false, false, false);
    }
}
