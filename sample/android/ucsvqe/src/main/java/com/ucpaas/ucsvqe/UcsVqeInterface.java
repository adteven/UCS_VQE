package com.ucpaas.ucsvqe;

import android.util.Log;

import java.nio.ByteBuffer;

/**
 * Created by vinton on 2017/08/10,0010.
 */

public class UcsVqeInterface {
    private static final String TAG = "UCSVQE";

    /* Support max 16kHz, per 10ms size = 2* 16000 / 100 */
    private static final int kMaxSampleBufferBytes = 320;

    private ByteBuffer mIn_neer;
    private ByteBuffer mIn_far;
    private ByteBuffer mOut_data;

    private static int mSampleRate = UcsVqeConfig.kUcsSampleRate8kHz;
    private static int mLengthPer10ms = 2 * (mSampleRate / 100);

    /**
     * Init UCS VQE module
     * @param sample_rate sample rate, 8k or 16k, @see UcsVqeConfig
     * @param config UCS VQE configuration
     * @return 0 if success, else failed
     */
    native int UcsVqeInit(int sample_rate, UcsVqeConfig config);

    /**
     * Destory UCS VQE module
     */
    native void UcsVqeClosed();

    native void UcsVqeSetDirectBufferAddress(ByteBuffer in_neer,
                                             ByteBuffer in_far,
                                             ByteBuffer out_data);

    /**
     * Process local capture audio data, every time only 10ms length
     * @param delay_ms always be 0
     * @return 0 if success, else failed
     */
    native int UcsVqeProc(int delay_ms);

    /**
     * Analysis remote render audio data, every time only 10ms length
     * @return 0 if success, else failed
     */
    native int UcsVqeFarend();

    /**
     * Enable/Disable speaker mode
     * @param enable true for speaker mode, false for earpiece mode
     * @return 0 if success, else failed
     */
    native int UcsVqeSetSpeakerEnable(boolean enable);

    private static final UcsVqeInterface ourInstance = new UcsVqeInterface();

    public static UcsVqeInterface getInstance() {
        System.loadLibrary("UCS_VQE");
        return ourInstance;
    }

    private UcsVqeInterface() {
        mIn_neer = ByteBuffer.allocateDirect(kMaxSampleBufferBytes);
        mIn_far = ByteBuffer.allocateDirect(kMaxSampleBufferBytes);
        mOut_data = ByteBuffer.allocateDirect(kMaxSampleBufferBytes);
    }

    /**
     * Init UCS VQE module
     * @param sample_rate sample rate, 8k or 16k, @see UcsVqeConfig
     * @param config UCS VQE configuration
     * @return 0 if success, else failed
     */
    public int UCSVQE_Init(int sample_rate, UcsVqeConfig config) {
        if (sample_rate != UcsVqeConfig.kUcsSampleRate8kHz
         && sample_rate != UcsVqeConfig.kUcsSampleRate16kHz) {
            Log.e(TAG, "UCSVQE_Init() failed. wrong sample_rate = " + sample_rate);
            return -1;
        }

        UcsVqeSetDirectBufferAddress(mIn_neer, mIn_far, mOut_data);

        mSampleRate = sample_rate;
        mLengthPer10ms = 2 * (mSampleRate / 100);
        return UcsVqeInit(sample_rate, config);
    }

    /**
     * Destory UCS VQE module
     */
    public void UCSVQE_Closed() {
        UcsVqeClosed();
    }

    /**
     * Process local capture audio data, every time only 10ms length
     * @param in_neer local capture audio data
     * @param out_data output audio data that has been processed by ucs vqe
     * @return 0 if success, else failed
     */
    public int UCSVQE_Process(byte[] in_neer, int delay_ms, byte[] out_data) {
        if (in_neer.length != mLengthPer10ms) {
            Log.e(TAG, "UCSVQE_Process() failed. in_neer length("
                    + in_neer.length + ") not match sample_rate = " + mSampleRate );
            return -1;
        }

        if (out_data.length < mLengthPer10ms) {
            Log.e(TAG, "UCSVQE_Process() failed. out_data length("
                    + out_data.length + ") too small.");
            return -1;
        }

        // The byte buffer must be rewinded since byteBuffer.position()
        // is increased at each call. If we don't do this,
        // next call would cause java.nio.BufferOverflowException.
        mIn_neer.rewind();
        mOut_data.rewind();

        mIn_neer.put(in_neer);
        int res = UcsVqeProc(delay_ms);
        if (res == 0) {
            mOut_data.get(out_data);
        }
        return res;
    }

    /**
     * Analysis remote render audio data, every time only 10ms length
     * @param in_far remote render audio data
     * @return 0 if success, else failed
     */
    public int UCSVQE_FarendAnalysis(byte[] in_far) {
        if (in_far.length != mLengthPer10ms) {
            Log.e(TAG, "UCSVQE_FarendAnalysis() failed. in_far length("
                    + in_far.length + ") not match sample_rate = " + mSampleRate );
            return -1;
        }

        // The byte buffer must be rewinded since byteBuffer.position()
        // is increased at each call. If we don't do this,
        // next call would cause java.nio.BufferOverflowException.
        mIn_far.rewind();
        mIn_far.put(in_far);
        return UcsVqeFarend();
    }

    /**
     * Enable/Disable speaker mode
     * @param enable true for speaker mode, false for earpiece mode
     * @return 0 if success, else failed
     */
    public int UCSVQE_SetSpeakerEnable(boolean enable) {
        return UcsVqeSetSpeakerEnable(enable);
    }
}
