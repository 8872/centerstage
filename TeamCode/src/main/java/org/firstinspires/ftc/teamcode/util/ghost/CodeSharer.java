package org.firstinspires.ftc.teamcode.util.ghost;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

public class CodeSharer {
  Context context;
  
  public CodeSharer(Context cxt) 
  {
    context=cxt;
  }
  
  public void share(String text) {
    Intent sharingIntent = new Intent(Intent.ACTION_SEND);
    sharingIntent.setType("text/plain");

    DateFormat dateFormat = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss");
    Date date = new Date();

    sharingIntent.putExtra(Intent.EXTRA_SUBJECT, "code_generated_"+dateFormat.format(date));
    sharingIntent.putExtra(Intent.EXTRA_TEXT, codeForString(text));
    context.startActivity(sharingIntent);
    }
    
  private String codeForString(String str) {
    String newString="\"";
    int charsInLine=0;
    for(int i=0;i<str.length();i++)
    {
      
      newString+=str.charAt(i);
      charsInLine+=1;
      if(charsInLine>=80)
      {
        newString+="\"+\n\"";
        charsInLine=0;
      }
    }
    return newString+"\"";
  }

  public void save(String content) {

      @SuppressLint("SdCardPath") File externalStorageDir = new File("/sdcard/FIRST/");
    @SuppressLint("SimpleDateFormat") DateFormat dateFormat = new SimpleDateFormat("MM-dd-yyyy_HH:mm:ss");
    Date date = new Date();
      File appDir = new File(externalStorageDir, "Ghost");
      if (!appDir.exists()) {
        appDir.mkdirs();
      }
      File file = new File(appDir, "code_generated_"+dateFormat.format(date) + ".mello");

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
