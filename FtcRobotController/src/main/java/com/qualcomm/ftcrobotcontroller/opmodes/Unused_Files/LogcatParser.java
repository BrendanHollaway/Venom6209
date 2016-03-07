/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.qualcomm.ftcrobotcontroller.opmodes.Unused_Files;

import java.io.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Scanner;

/**
 *
 * @author Brendan Hollaway
 */
public class LogcatParser {
    public static void main(String[] args) throws IOException
    {
        Scanner sc = new Scanner(new File("com.qualcomm.ftcrobotcontroller.logcat"));
        DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        Date date = new Date();
        PrintWriter pw = new PrintWriter(new File("Old Patterns. Time= " + date.getTime()));
        System.out.println("hello");
        sc.useDelimiter(".*?My Name is Bo, Run #");
        ArrayList<String> arr = new ArrayList<String>();
        int[][][] patterns = new int[18][80][60];
        if(sc.hasNext())
        {
            sc.next();
        }
        while(sc.hasNext())
        {
            String next = sc.next();
            String output = "";
            if(next.length() < 8)
                next= sc.next();
            int z = Integer.parseInt(next.substring(0, 2));
            if(z > patterns.length)
                break;
            int i = 0;
            int r = Integer.parseInt(next.substring(3, 8)) / 80;
            int c = Integer.parseInt(next.substring(3, 8)) % 80;
            while (next.charAt(i) == 'a' || next.charAt(i) == 'p') {
                if (next.charAt(i) == 'a') {
                    r++;
                    c = 0;
                } else //adds a pixel to the array
                   // patterns[z][r][c] = Inte
                    //patterns[z][r][c++] = Integer.parseInt(next.substring(i, i + 3)) << 16 + Integer.parseInt(next.substring(i + 3, i + 6)) << 8 + Integer.parseInt(next.substring(i + 6, i + 9));
                i++;
            }
        }

        int cnt = 0;
        for(int[][] arr1 : patterns)
        {
            String output = "Arr #" + ++cnt;
            for (int[] arr2 : arr1)
            {
                output += "[";
                for (int i : arr2)
                    output += String.format("%07d,", i);
                output += "]\n";
            }
            output += "\n\n";
            pw.write(output);
            pw.flush();
        }
        pw.close();

            /*while(!next.substring(i,i+++3).matches("[RGBa]{3}"));
            i--;
            while(next.charAt(i) == 'R' || next.charAt(i) == 'G' || next.charAt(i) == 'B' || next.charAt(i) == 'a')
                output += next.charAt(i++);
            System.out.println(output.length() + " " + output);
            arr.add(output);
            z++;*/
        /*char[][][] patterns = new char[arr.size()][80][60];
        //create array of patterns
        for(int i = 0, j=0; i < arr.size(); i++, j=0)
        {
            String output = "Array #" + i + "\n";
            System.out.println(i);
            for(int r = 0; r < patterns[0].length; r++)
            {
                Arrays.fill(patterns[i][r], 'e');
                output += "[";
                for(int c = 0; c < patterns[0][0].length && j < arr.get(i).length() && arr.get(i).charAt(j) != 'a'; c++, j++)
                {
                    patterns[i][r][c] = arr.get(i).charAt(j);
                    output += patterns[i][r][c];
                }
                j++;
                /*int c = 0;
                System.out.println(arr.get(i).charAt(r * 50 + c));
                while(arr.get(i).charAt(r * 50 + c) != 'R' && c < 80)
                {
                    //System.out.println(arr.get(i).charAt(r * 50 + c));
                    patterns[i][r][c] = "" + arr.get(i).charAt(r * 50 + c++);
                }
                while(arr.get(i).charAt(r * 50 + c) == 'R' && c < 80)
                {
                    patterns[i][r][c] = "" + arr.get(i).charAt(r * 50 + c++);
                }*/
                /*
                output += "]\n";
            }
            output += "\n\n";
            System.out.print(output);*/
        //}
        pw.close();
        
    }
    
}
