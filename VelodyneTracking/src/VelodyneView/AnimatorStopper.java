package VelodyneView;

import javax.media.opengl.GLAnimatorControl;

public class AnimatorStopper extends Thread{
	private GLAnimatorControl anim;
	
	public AnimatorStopper(GLAnimatorControl anim) {
		this.anim = anim;
	}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		if(anim.isStarted()) anim.stop();
		System.exit(0);
	}
}