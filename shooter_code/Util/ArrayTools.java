package Util;

public class ArrayTools {
	public static double[] multiply(double[] a, double scale){
		double[] b=new double[a.length];
		for(int i=0;i<a.length;i++) {
			b[i]=a[i]*scale;
		}
		return b;
	}
	public static double[] copy(int[] a){
		double[] b = new double[a.length];
		for(int i=0;i<a.length;i++){
			b[i]=a[i];
		}
		return b;
	}
	public static double[] add(double[] a, double c){
		double[] b = new double[a.length];
		for(int i=0;i<a.length;i++){
			b[i]=a[i]+c;
		}
		return b;
	}
}
