package org.firstinspires.ftc.teamcode.util.ghost;

import android.annotation.SuppressLint;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class GhostRecorder {
  private String instructions = "";
  private int updatesSinceValueChanged = 0;
  private StickValues stickValues = new StickValues();
  private ButtonValues buttonValues = new ButtonValues();
  private TriggerValues triggerValues = new TriggerValues();  // Added TriggerValues

  public GhostRecorder() {
  }

  public void recordLeftStickY(double lsticky) {
    stickValues.setValue(StickValues.leftStickY, lsticky);
  }

  public void recordRightStickY(double rsticky) {
    stickValues.setValue(StickValues.rightStickY, rsticky);
  }

  public void recordLeftStickX(double lstickx) {
    stickValues.setValue(StickValues.leftStickX, lstickx);
  }

  public void recordRightStickX(double rstickx) {
    stickValues.setValue(StickValues.rightStickX, rstickx);
  }

  public void recordButtonX(boolean val) {
    buttonValues.setValue(ButtonValues.buttonX, val);
  }

  public void recordButtonY(boolean val) {
    buttonValues.setValue(ButtonValues.buttonY, val);
  }

  public void recordButtonA(boolean val) {
    buttonValues.setValue(ButtonValues.buttonA, val);
  }

  public void recordButtonB(boolean val) {
    buttonValues.setValue(ButtonValues.buttonB, val);
  }

  public void recordDpadUp(boolean val) {
    buttonValues.setValue(ButtonValues.dpadUp, val);
  }

  public void recordDpadDown(boolean val) {
    buttonValues.setValue(ButtonValues.dpadDown, val);
  }

  public void recordDpadLeft(boolean val) {
    buttonValues.setValue(ButtonValues.dpadLeft, val);
  }

  public void recordDpadRight(boolean val) {
    buttonValues.setValue(ButtonValues.dpadRight, val);
  }

  public void recordBumperLeft(boolean val) {buttonValues.setValue(ButtonValues.bumperLeft, val);}

  public void recordBumperRight(boolean val) {buttonValues.setValue(ButtonValues.bumperRight, val);}

  public void recordLeftTrigger(double val) {
    triggerValues.setValue(TriggerValues.leftTrigger, val);
  }

  public void recordRightTrigger(double val) {
    triggerValues.setValue(TriggerValues.rightTrigger, val);
  }

  public String getStringOfChangedVals(ControllerValues vals) {
    String line = "";
    @SuppressWarnings("unchecked")
    ArrayList<String> syms = vals.getSymbolsOfChanged();
    for (int i = 0; i < syms.size(); i++) {
      line += syms.get(i) + ":" + vals.getValue(syms.get(i));
      line += " ";
    }
    return line;
  }

  public void update() {
    String line = getStringOfChangedVals(stickValues) + getStringOfChangedVals(buttonValues) +
            getStringOfChangedVals(triggerValues);  // Added trigger values
    if (!line.equals("")) {
      if (updatesSinceValueChanged > 0) {
        instructions += String.valueOf(updatesSinceValueChanged) + " ";
      }
      instructions += line;
      updatesSinceValueChanged = 0;
    }

    updatesSinceValueChanged += 1;
  }

  public String getString() {
    return instructions + String.valueOf(updatesSinceValueChanged);
  }

  public void save(String content) {
    File externalStorageDir = new File("/sdcard/FIRST/");
    DateFormat dateFormat = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss");
    Date date = new Date();
    File appDir = new File(externalStorageDir, "Ghost");
    if (!appDir.exists()) {
      appDir.mkdirs();
    }
    File file = new File(appDir, "code_generated_" + dateFormat.format(date) + ".mello");

    try {
      FileWriter writer = new FileWriter(file);
      writer.append(content);
      writer.flush();
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void save(String content, String name) {
    File externalStorageDir = new File("/sdcard/FIRST/");
    DateFormat dateFormat = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss");
    Date date = new Date();
    File appDir = new File(externalStorageDir, "Ghost");
    if (!appDir.exists()) {
      appDir.mkdirs();
    }
    File file = new File(appDir, name + ".mello");

    try {
      FileWriter writer = new FileWriter(file);
      writer.append(content);
      writer.flush();
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}


