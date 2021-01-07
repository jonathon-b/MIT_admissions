package gameObject;

import Test.DrawShip;
import Util.GDV5;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Random;

public class ShipEnemy extends Enemy{
	int limit, iterations=0;
	static Random r = new Random();

	public ShipEnemy(double x, double y, double dx, double dy, double size, GDV5 g) {
		super(x, y, dx, dy, size);
		int xlimit;
		if(dx<0)
			xlimit=(int)(x/(-dx));
		else
			xlimit=(int)((g.getWidth()-this.getMaxX())/dx);
		limit=r.nextInt(Math.min((int)(g.getHeight()/this.dy-1),xlimit));
	}
	public void shoot(float bound, ArrayList b,Ship s){
		double deltaY=s.hitBox.getCenterY()-this.getCenterY(), deltaX=s.hitBox.getCenterX()-this.getCenterX();
		double speed=6;
	if(r.nextFloat()>bound)
		b.add(new Bullet(new Point2D.Double(this.getCenterX(),this.getCenterY()),Math.sin(Math.atan2(deltaY,deltaX))*speed,Math.cos(Math.atan2(deltaY,deltaX))*speed,s,true,5));
	}
	public void update(){
		iterations++;
		if(this.iterations<this.limit)
			super.update();
	}
}
