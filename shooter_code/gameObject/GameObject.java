package gameObject;

import java.awt.*;
import java.awt.geom.Rectangle2D;

public class GameObject extends Rectangle2D.Double {
	public double dx,dy;

	public GameObject(double x, double y, double dx, double dy, double size){
		super(x,y,size, size);
		this.dx=dx; this.dy=dy;
	}

	public void update(){
		this.x+=dx; this.y+=dy;
	}

	public void drawBound(Graphics2D win, Color c){
		win.setColor(c);
		win.draw(this);
	}
}
