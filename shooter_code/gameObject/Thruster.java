package gameObject;

public class Thruster extends Particle {

	public Thruster(double x, double y, double size, int maxMovement, double angle, double distribution) {
		super(x, y, size, maxMovement);
		this.dy=2*Math.cos(angle)+distribution*Math.abs(r.nextGaussian());
		this.dx=2*-Math.sin(angle)+distribution*Math.abs(r.nextGaussian());
	}
}
