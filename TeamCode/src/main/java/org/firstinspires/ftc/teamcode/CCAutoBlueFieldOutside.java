package org.firstinspires.ftc.teamcode;

public class CCAutoBlueFieldOutside extends CCAutoCommon {
    public CCAutoBlueFieldOutside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(false, false);
    }
}
