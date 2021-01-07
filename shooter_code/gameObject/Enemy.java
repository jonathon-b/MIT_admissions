package gameObject;

import Test.DrawShip;
import Util.GDV5;

import java.awt.*;
import java.awt.geom.Rectangle2D;

public class Enemy extends GameObject{

	public boolean kill=false;

	public Enemy(double x, double y, double dx, double dy, double size) {
		super(x, y, dx, dy, size);
	}

	public void collisionCheck(Bullet bullet){
		if(GDV5.collides(this,bullet))
			this.killEnemy(this.getCenterX(),this.getCenterY());
	}

	public void killEnemy(double x, double y){
		this.kill=true;
		DrawShip.explosions.add(new ExplosionArray(x,y,3,20,40,2, Color.MAGENTA));
		DrawShip.addRumble(15);
		//if(!DrawShip.sd.isPlaying(1))
		DrawShip.sd.play(1);
	}

	public boolean isDead(){return kill;}

}
