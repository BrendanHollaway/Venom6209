/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.qualcomm.ftcrobotcontroller;


/**
 *
 * @author Brendan Hollaway
 */
public class DPoint extends Point
{
    double x, y;
    public DPoint(double a, double b)
    {
        x = a;
        y = b;
    }
    //@Override
    public double getX()
    {
        return x;
    }
    //@Override
    public double getY()
    {
        return y;
    }
    public static Point toPoint(DPoint d)
    {
        return new Point((int)Math.floor(d.x), (int)Math.floor(d.y));
    }
    public static Point toMaxXPoint(DPoint d)
    {
        return new Point((int)Math.ceil(d.x), (int)Math.floor(d.y));
    }
    public static Point toMaxYPoint(DPoint d)
    {
        return new Point((int)Math.floor(d.x), (int)Math.ceil(d.y));
    }
    public static Point toMaxPoint(DPoint d)
    {
        return new Point((int)Math.ceil(d.x), (int)Math.ceil(d.y));
    }
    public static DPoint makeDPoint(org.opencv.core.Point p)
    {
        return new DPoint(p.x, p.y);
    }
    @Override
    public String toString()
    {
        return String.format("x=%.2f, y=%.2f; ", x, y);
    }

}