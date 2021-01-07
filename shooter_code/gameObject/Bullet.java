package gameObject;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Bullet extends Rectangle2D.Double {
	public static final double baseVel=5;
	public double dx,dy;
	boolean killsEnemies;
	public boolean kill=false;
	public Bullet(Point2D.Double p, double sin, double cos, Ship s, boolean velControl, double size){
		super(p.getX()-(size/2),p.getY(),size,size);
		if(!velControl) {
			this.dx = baseVel * sin + s.dx;
			this.dy = -baseVel * cos + s.dy;
		}
		else{
			this.dx=cos;
			this.dy=sin;
		}
		killsEnemies=!velControl;
		}
	public void update(){
		this.x+=dx;this.y+=dy;
	}
	public boolean kill(){
		if(this.getMaxY()<0||kill)
			return true;
		return false;
	}
	public void explode(ArrayList al){
		al.add(new ExplosionArray(this.getCenterX(),this.getCenterY(),3,20,40,4,Color.MAGENTA));
	}
	public boolean killEnemies(){
		return killsEnemies;
	}
	public void draw(Graphics2D win, Color c){
		win.setColor(Color.WHITE);
		win.draw(this);
		win.setColor(c);
		win.fill(this);
	}
}
