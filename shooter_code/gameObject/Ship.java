package gameObject;

import Util.ArrayTools;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Arc2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.lang.reflect.Array;
import java.util.Arrays;

public class Ship extends Polygon{

	public AffineTransform at=new AffineTransform();
	public double[] xPoints={2,4,6,5,4,3,2,1,0};
	public double[] yPoints={1,1,5,6,5,6,5,6,5};
	public Point2D.Double[] startGun= new Point2D.Double[2];
	public Point2D.Double[] rotatedGun= new Point2D.Double[2];
	public Point2D.Double rotatedThrust[]=new Point2D.Double[2];
	double anchorx;
	double anchory;
	public final static double angleStep = Math.PI/50;
	static public int[] placeholder={2,4,6,5,4,3,2,1,0};
	private Arc2D.Double a;
	private double aHeight=2,aWidth=2,aX=2,aY=0;
	public static final double baseVel=6;
	public double dx=0,dy=0;
	public int lives;
	double intScale=5;
	public Rectangle2D.Double[] gun= new Rectangle2D.Double[2];
	double maxWidth,maxHeight;
	public double angle = 0;
	public Rectangle2D.Double hitBox;
	private Double minX,minY,maxX,maxY;
	private Color[] rotatedColors = {Color.LIGHT_GRAY,Color.YELLOW,Color.YELLOW,Color.GRAY};
	private Shape[] rotated=new Shape[4];

	public Ship(double scale, double maxHeight, double maxWidth, double xOffset, double yOffset, int lives){
		super(placeholder,placeholder,placeholder.length);
		this.lives=lives;
		internalScale(scale);
		this.intScale=scale;
		this.maxHeight=maxHeight;
		this.maxWidth=maxWidth;
		offset(xOffset,yOffset);
		makePolygon();
		makeArc();
		makeGun();
		hitBox=new Rectangle2D.Double();
	}

	public boolean isAlive(){
		return this.lives>0;
	}

	public void offset(double x, double y){
		xPoints=ArrayTools.add(xPoints,x);
		yPoints=ArrayTools.add(yPoints,y);
	}

	private void internalScale(double scale){
		xPoints=ArrayTools.multiply(xPoints,scale);
		yPoints=ArrayTools.multiply(yPoints,scale);
		aX*=scale;
		aY*=scale;
		aWidth*=scale;
		aHeight*=scale;
	}

	public void resetVel(){
		this.dx=0;
		this.dy=0;
	}

	private void makeGun(){
		double gunWidth=0.3*intScale;
		double gunHeight=1.5*intScale;
		gun[0]=new Rectangle2D.Double(xPoints[0]-(gunWidth/2)+2,yPoints[0]-gunHeight,gunWidth,gunHeight);
		gun[1]=new Rectangle2D.Double(xPoints[1]-(gunWidth/2)-1,yPoints[0]-gunHeight,gunWidth,gunHeight);
	}

	private void makeArc(){
		a=new Arc2D.Double(xPoints[0],yPoints[0]-1*intScale,aWidth,aHeight,0,180,Arc2D.PIE);
}

	private void makePolygon(){
		this.reset();
		for(int i=0;i<xPoints.length;i++) {
			this.addPoint((int)xPoints[i],(int)yPoints[i]);
		}
	}


	public void update(){

		this.angle%= 2*Math.PI;

		if(xPoints[2]+dx>=maxWidth) {
			dx = maxWidth - xPoints[2];
			dy=0;
		}
		if(xPoints[xPoints.length-1]+dx<=0) {
			dx = 0 - xPoints[xPoints.length - 1];
			dy=0;
		}
		if(gun[0].getY()+dy<=0) {
			dy = 0 - gun[0].getY();
			dx=0;
		}
		if(yPoints[3]+dy>=maxHeight) {
			dy = maxHeight - yPoints[3];
			dx=0;
		}
		xPoints=ArrayTools.add(xPoints,dx);
		yPoints=ArrayTools.add(yPoints,dy);
		for(int i=0;i<xPoints.length;i++){
			xpoints[i]=(int)xPoints[i];
			ypoints[i]=(int)yPoints[i];
		}

		gun[0].x+=dx;
		gun[0].y+=dy;
		gun[1].x+=dx;
		gun[1].y+=dy;

		a.x+=dx;
		a.y+=dy;
		minY=a.getY();
		minX=xPoints[xPoints.length-1];
		maxX=xPoints[2];
		maxY=yPoints[3];
		hitBox.x=minX;
		hitBox.y=minY;
		hitBox.width=maxX-minX;
		hitBox.height=maxY-minY;
		angle%=Math.PI*2;

		anchorx=hitBox.getCenterX();
		anchory= hitBox.getCenterY();

		Point2D.Double thrustPoints[]=new Point2D.Double[2];
		thrustPoints[0]=new Point2D.Double(xPoints[6],yPoints[6]);
		thrustPoints[1]=new Point2D.Double(xPoints[4],yPoints[6]);
		at.getRotateInstance(angle, anchorx,anchory).transform(thrustPoints,0,rotatedThrust,0,thrustPoints.length);

		rotated[0]=at.getRotateInstance(angle,anchorx,anchory).createTransformedShape(this);
		rotated[1]=at.getRotateInstance(angle,anchorx,anchory).createTransformedShape(gun[0]);
		rotated[2]=at.getRotateInstance(angle,anchorx,anchory).createTransformedShape(gun[1]);
		rotated[3]=at.getRotateInstance(angle,anchorx,anchory).createTransformedShape(a);

		startGun[0]=new Point2D.Double(gun[0].getCenterX(),gun[0].getMinY());
		startGun[1]=new Point2D.Double(gun[1].getCenterX(),gun[1].getMinY());
		at.getRotateInstance(angle, anchorx,anchory).transform(startGun,0,rotatedGun,0,startGun.length);
	}

	public void fill(Graphics2D win){


		for(int i=0;i<rotated.length;i++){
			win.setColor(rotatedColors[i]);
			if(rotated[i]!=null)    {win.fill(rotated[i]);}
		}

/*
		win.setColor(Color.RED);
		win.draw(hitBox);
*/
	}
}
