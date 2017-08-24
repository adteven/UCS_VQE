package com.ucpaas.ucsvqe;

/**
 * Created by vinton on 2017/08/10,0010.
 */

public class UcsVqeConfig {
    // sample rate
    public final static int kUcsSampleRate8kHz = 8000;
    public final static int kUcsSampleRate16kHz = 16000;

    // aec Non-linear level

    public final static int kUcsEcSuppressionDefault = 0;
    /* only for Non-mobile */
    public final static int kUcsEcSuppressionLow = 1;
    public final static int kUcsEcSuppressionModerate = 2;
    public final static int kUcsEcSuppressionHigh = 3;
    /* only for mobile */
    public final static int kUcsEcSuppressionVeryHigh = 4;


    // noise suppression level
    public final static int kUcsNsDefault = 0;
    public final static int kUcsNsLow = 1;
    public final static int kUcsNsModerate = 2;
    public final static int kUcsNsHigh = 3;
    public final static int kUcsNsVeryHigh = 4;

    // agc level
    public final static int kUcsAgcDefault = 0;
    public final static int kUcsAgcLow = 1;
    public final static int kUcsAgcModerate = 2;
    public final static int kUcsAgcHigh = 3;

    public boolean aec_enable;
    public boolean agc_enable;
    public boolean ns_enable;
    public int aec_level;
    public int agc_level;
    public int ns_level;

    public UcsVqeConfig() {
        aec_enable = true;
        agc_enable = true;
        ns_enable = true;
        aec_level = kUcsEcSuppressionDefault;
        agc_level = kUcsAgcDefault;
        ns_level = kUcsNsDefault;
    }
}
