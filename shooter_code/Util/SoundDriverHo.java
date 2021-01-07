package Util;

import java.awt.Canvas;
import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.FloatControl;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;

/**
 * A class that stores and plays sounds.
 * <br><br>Upgrades:<list><li>Methods to play a song, loop, etc. can be called with a String referring to the original string input when instantiating the SoundDriver instead of an index.
 * <li>Works outside of the default package and works with resources.
 * <li>The volume actually works now.
 * <li>Added methods to set the volume to every sound or to stop all sounds in the SoundDriver.
 * <br><br>Upgrades by Devin Ho
 * 
 * @author bhsjava (Original Code)
 * @author Devin Ho (For Upgrades)<list><li>Class of 2019<li>Computer Science A
 */


public class SoundDriverHo {
	
	private Clip[] clips;
    private String[] names;
    private int[] framePosition;
    private FloatControl[] gainControl;
    private Canvas game;
    
    /**
     * Creates a SoundDriverUpgraded given the sound locations. These strings can be reused to call all other methods instead of an index.
     * @param aClips
     */
    public SoundDriverHo(String[] aClips, Canvas game) {

    	this.game = game;
    	
    	System.out.println("<Sound Driver> LOADING SOUNDS");
    	
    	this.names = aClips;
    	
    	clips = new Clip[aClips.length];
        gainControl = new FloatControl[aClips.length];
        framePosition = new int[aClips.length];
        try {
            AudioFormat format = new AudioFormat(AudioFormat.Encoding.PCM_SIGNED,
                    AudioSystem.NOT_SPECIFIED,
                    16, 2, 4,
                    AudioSystem.NOT_SPECIFIED, true);
            DataLine.Info info = new DataLine.Info(Clip.class, format);
            
            for (int i = 0; i < clips.length; i++) {
                //File soundFile = new File(aClips[i]);
                //BufferedInputStream bs = new BufferedInputStream(new FileInputStream(soundFile));
            	//URL inS = ;
            	InputStream resource = game.getClass().getResourceAsStream(aClips[i]);
            	BufferedInputStream stream = new BufferedInputStream(resource);
            	AudioInputStream soundIn = AudioSystem.getAudioInputStream(stream);
                clips[i] = (Clip) AudioSystem.getLine(info);
                //clips[i] = AudioSystem.
                clips[i].open(soundIn);
                gainControl[i] = (FloatControl) clips[i].getControl(FloatControl.Type.MASTER_GAIN);
                System.out.println("<Sound Driver> " + aClips[i]);
            }
            //System.out.println("Audio File Loaded");
        } catch (UnsupportedAudioFileException ex) {
            System.out.println("Unsupported Audio File");
        } catch (LineUnavailableException ex) {
            System.out.println("Line Unavailable");
        } catch (IOException ex) {
            System.out.println("IO Error" + ex);
        }
        
        System.out.println("<Sound Driver> SOUNDS LOADED");
    }

    public int getIndex(String clipName) {
    	for(int i = 0; i < this.names.length; i++) {
    		if(this.names[i].equals(clipName)) return i;
    	}
    	System.out.println("<Sound Driver> String is not an index!");
    	return -1;
    }
    
    public int numSongs() {
    	return this.clips.length;
    }
    
    public String getString(int index) {
    	return this.names[index];
    }
    
    public void play(int value) {
        clips[value].stop();
        clips[value].setFramePosition(0);
        clips[value].start();
    }
    public void play(String clipName) {
    	this.play(this.getIndex(clipName));
    }

    public void loop(int value) {
        clips[value].stop();
        clips[value].setFramePosition(0);
        clips[value].loop(Clip.LOOP_CONTINUOUSLY);        
    }
    public void loop(String clipName) {
    	this.loop(this.getIndex(clipName));
    }

    public void stop(int value) {
        clips[value].stop();
    }
    public void stop(String clipName) {
    	this.stop(this.getIndex(clipName));
    }
    
    public void pause(int value){
        framePosition[value] = clips[value].getFramePosition();
        clips[value].stop();        
    }
    public void pause(String clipName) {
    	this.pause(this.getIndex(clipName));
    }
    
    public void resume(int value){
        clips[value].setFramePosition(framePosition[value]);
        clips[value].start();
    }
    public void resume(String clipName) {
    	this.resume(this.getIndex(clipName));
    }
    
    public boolean isPlaying(int value){
        return clips[value].isRunning();
    }
    public boolean isPlaying(String clipName) {
    	return this.isPlaying(this.getIndex(clipName));
    }

    public void setVolume(int clipIndex, float volume) {
        gainControl[clipIndex].setValue(volume);
    }
    public void setVolume(String clipName, float volume) {
    	this.setVolume(this.getIndex(clipName), volume);
    }

	public void setVolumeAll(float f) {
		// TODO Auto-generated method stub
		for(int i = 0; i < this.clips.length; i++) {
			this.setVolume(i, f);
		}
	}

	public void stopAll() {
		// TODO Auto-generated method stub
		for(int i = 0; i < this.clips.length; i++) {
			if(this.isPlaying(i)) {
				this.stop(i);
			}
		}
	}
    
	
	
	

}
