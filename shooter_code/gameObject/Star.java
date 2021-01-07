package gameObject;

import java.awt.*;
import java.awt.geom.Rectangle2D;
import java.util.Random;

public class Star extends Rectangle2D.Double {
	public static Random r = new Random();
	private double dy;
	static double dx;
	int screenHeight;
	Ship s;

	public Star(int maxSize,int screenWidth,int maxDy, int screenHeight,Ship s){
		this.screenHeight=screenHeight;
		this.width=r.nextInt(maxSize)+2;
		this.height=this.width;
		this.x=r.nextInt(screenWidth-(int)this.width);
		this.y=0;
		this.dy=r.nextInt(maxDy-1)+1;
		this.s=s;
	}

	public boolean kill(){
		return this.y+this.height>=2*screenHeight;
	}

	private void moveStar(){
		if(this.getX()>=1280){
			this.x=0;
		}
		if(this.getMaxX()<=0){
			this.x=1280-this.getWidth();
		}
	}

	public void update(){
		moveStar();
		this.y-=s.dy-this.dy;
		this.x-=s.dx;
	}


	public void fill(Graphics2D win, Color c){
		win.setColor(new Color(241, 196, 15));

		win.fill(this);
	}

}
