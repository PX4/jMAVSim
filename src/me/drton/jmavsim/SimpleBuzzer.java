package me.drton.jmavsim;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;

import me.drton.jmavlib.mavlink.MAVLinkDataType;
import me.drton.jmavlib.mavlink.MAVLinkField;
import me.drton.jmavlib.mavlink.MAVLinkMessage;

public class SimpleBuzzer implements Peripherial {
    
    static final int freqMavLinkOffsetInBytes = 2;
    static final int durationMavLinkOffsetInBytes = 6;
    static final int silenceMavLinkOffsetInBytes = 10;
    
    BlockingQueue<Note> notes = new LinkedBlockingQueue<>();
    
    AudioFormat af;
    SourceDataLine sdl;
    float sample_rate = 44100; // assume a sample rate of 44.1 kHz

    class Note {
    	
        public int frequency;
        public int duration;
        public int silence;
        
        public Note(int frequency, int duration, int silence) {
            this.frequency = Integer.valueOf(frequency);
            this.duration = duration;
            this.silence = silence;            
        }
    }

    public class NoteConsumer implements Runnable {
        public void run() {
            while (true) {
                try {
                    Note timedNote = notes.take();
                    if(timedNote.frequency > 0) {
                        generateTone(timedNote.frequency, timedNote.duration);
                    }
                    if(timedNote.silence > 0) {
                    	Thread.sleep(timedNote.silence);
                    }
                    
                } catch (LineUnavailableException e) {
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public SimpleBuzzer() {
        Thread myThread = new Thread(new NoteConsumer());
        af = new AudioFormat(sample_rate, 8, 1, true, false);
        try {
            sdl = AudioSystem.getSourceDataLine(af);
            sdl.open(af);
        } catch (LineUnavailableException e) {
            e.printStackTrace();
        }

        sdl.start();

        myThread.start();
    }

    public void generateTone(int hz, int msecs) throws LineUnavailableException {
        byte[] buf = new byte[2048];

        int k = 0;
        
        for (int i = 0; i < msecs * sample_rate / 1000;) {
            // fill up the local buffer
            for (k = 0; k < 2047 && (i < msecs * sample_rate / 1000); i++, k++) {
                buf[k] = (byte)(Math.sin(i / (sample_rate / hz) * 2.0 * Math.PI) * 100);
            }

            sdl.write(buf, 0, k);
        }

        sdl.drain();
    }
	
    public void filterMessage(MAVLinkMessage msg) {
        if("PLAY_TUNE".equals(msg.getMsgName())) {

            ByteBuffer bb = ByteBuffer.wrap((byte[]) msg.
                            get(new MAVLinkField(MAVLinkDataType.UINT8, 30, "tune"))).
                            order(ByteOrder.LITTLE_ENDIAN);                        
            
            notes.add(new Note(bb.getInt(freqMavLinkOffsetInBytes), 
                               bb.getInt(durationMavLinkOffsetInBytes)/1000, 
                               bb.getInt(silenceMavLinkOffsetInBytes)/1000));
        }
    }
}
