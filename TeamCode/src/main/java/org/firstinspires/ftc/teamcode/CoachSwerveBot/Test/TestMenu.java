package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.util.ArrayList;

public class TestMenu {

    final public ArrayList<String> menuItems = new ArrayList<>();
    private final ImprovedGamepad impGamepad1;
    private final Telemetry telemetry;
    private int currentItem = 0;
    private int selectedItem = -1;

    public TestMenu(ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.impGamepad1 = _impGamepad1;
        this.telemetry = _telemetry;
    }

    public boolean update() {
        selectedItem = -1;
        telemetry.addData("Press B", "to select item");
        for (int ii=0; ii<menuItems.size(); ii++) {
            telemetry.addData(Integer.toString(ii), menuItems.get(ii) + (ii==currentItem ? "<--": " "));
        }
        if (impGamepad1.a.isInitialPress()) {
            currentItem++;
            if (currentItem >= menuItems.size()) {
                currentItem = 0;
            }
        }
        if (impGamepad1.y.isInitialPress()) {
            currentItem--;
            if (currentItem < 0) {
                currentItem = menuItems.size()-1;
            }
        }
        if (impGamepad1.b.isInitialPress()) {
            selectedItem = currentItem;
            return true;
        }

        telemetry.update();
        return false;
    }

    public String getSelectedItemName() {
        return menuItems.get(selectedItem);
    }

    public int getSelectedItemIndex() {
        return selectedItem;
    }

    public void addItem(String testCaseName) {
        menuItems.add(testCaseName);
    }

    public boolean isItemSelected() {
        return selectedItem >= 0;
    }
}
