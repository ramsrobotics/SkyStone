package org.firstinspires.ftc.teamcode;


public class CCAutoRedFieldInside extends CCAutoCommon {
    public CCAutoRedFieldInside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() {
        runAuto(true, false, false);
    }
}
