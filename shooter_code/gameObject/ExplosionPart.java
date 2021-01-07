package gameObject;

import java.util.Random;

public class ExplosionPart extends Particle {

	static Random r = new Random();

	public ExplosionPart(double x, double y, double size, int maxMovement, double distribution) {
		super(x, y, size, maxMovement);
		this.dx=r.nextGaussian()*distribution;
		this.dy=r.nextGaussian()*distribution;
	}
}
