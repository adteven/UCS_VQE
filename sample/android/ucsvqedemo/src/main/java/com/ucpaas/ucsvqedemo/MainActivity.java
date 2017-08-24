package com.ucpaas.ucsvqedemo;

import android.os.Bundle;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.widget.TextView;

import com.ucpaas.ucsvqe.UcsVqeConfig;
import com.ucpaas.ucsvqe.UcsVqeInterface;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class MainActivity extends AppCompatActivity {
    public TextView txtTips = null;
    private FileOutputStream fosOutData = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        txtTips = (TextView) findViewById(R.id.txtTips);

        UcsVqeTest();
    }

    public byte[] convertStream2byteArrry(String filepath) {
        InputStream inStream = null;
        try {
            inStream = this.getResources().getAssets().open(filepath);
        } catch (IOException e2) {
            // TODO Auto-generated catch block
            e2.printStackTrace();
        }

        if (inStream == null) {
            return null;
        }

        int length = 0;
        try {
            length = inStream.available();
        } catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }

        byte[] buffer = new byte[length];
        try {
            inStream.read(buffer);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
//        Log.i(TAG, "buffer.length = " + buffer.length);

        return buffer;
    }

    private void UcsVqeTest() {
        txtTips.setText(R.string.tip_loader);
        UcsVqeConfig config = new UcsVqeConfig();
        UcsVqeInterface.getInstance().UCSVQE_Init(UcsVqeConfig.kUcsSampleRate8kHz,
                config);
        txtTips.setText(R.string.tip_ready);
        txtTips.setText(R.string.tip_processing);

        byte[] in_neer = convertStream2byteArrry("near8k.pcm");
        byte[] in_far = convertStream2byteArrry("far8k.pcm");
        int bytesPer10ms = 2 * UcsVqeConfig.kUcsSampleRate8kHz / 100;
        byte[] tmpBuf = new byte[bytesPer10ms];
        byte[] out_data = new byte[bytesPer10ms];
        int readneer = 0;
        int readfar = 0;
        try {
            String path = Environment.getExternalStorageDirectory().getPath();
            String outpath = path + "/aec_out.pcm";
            fosOutData = new FileOutputStream(outpath);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        if (in_neer == null) {
            txtTips.setText(R.string.tip_in_neer_open_failed);
            return;
        }
        if (in_far == null) {
            txtTips.setText(R.string.tip_in_far_open_failed);
        }
        if (fosOutData == null) {
            txtTips.setText(R.string.tip_out_data_open_failed);
            return;
        }
        while (readneer < in_neer.length &&
                readfar < in_far.length) {
            System.arraycopy(in_far, readfar, tmpBuf, 0, bytesPer10ms);
            UcsVqeInterface.getInstance().UCSVQE_FarendAnalysis(tmpBuf);
            readfar += bytesPer10ms;

            System.arraycopy(in_neer, readneer, tmpBuf, 0, bytesPer10ms);
            UcsVqeInterface.getInstance().UCSVQE_Process(tmpBuf, 0, out_data);
            readneer += bytesPer10ms;

            try {
                fosOutData.write(out_data, 0, bytesPer10ms);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        txtTips.setText(R.string.tip_done);
        UcsVqeInterface.getInstance().UCSVQE_Closed();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        UcsVqeInterface.getInstance().UCSVQE_Closed();
    }
}
