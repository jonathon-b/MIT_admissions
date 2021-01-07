package gameObject;

import java.awt.*;
import java.awt.geom.Rectangle2D;
import java.util.Random;

public class Particle extends Rectangle2D.Double {
	static Random r = new Random();
	double dy,dx;
	int count =0;
	int maxMovement;

	public Particle(double x, double y, double size, int maxMovement){
		this.maxMovement=maxMovement;
		this.x=x;
		this.y=y;
		this.width=size;
		this.height=size;
	}

	public boolean kill(){
		return count>=maxMovement;
	}

	public void update(){
		this.x+=dx;
		this.y+=dy;
		count++;
	}

	public void fill(Graphics2D win, Color c){
		win.setColor(c);
		win.fill(this);
	}

}
