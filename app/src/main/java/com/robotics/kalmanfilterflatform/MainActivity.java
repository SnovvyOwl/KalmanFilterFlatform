package com.robotics.kalmanfilterflatform;


import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {
    private TextView Ax_Val;
    private TextView Ay_Val;
    private TextView Az_Val;
    private TextView Gx_Val;
    private TextView Gy_Val;
    private TextView Gz_Val;
    private TextView Mx_Val;
    private TextView My_Val;
    private TextView Mz_Val;
    private SensorEventListener mlistener;
    private SensorManager mSensorManager;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        Ax_Val=findViewById(R.id.Ax_val);
        Ay_Val=findViewById(R.id.Ay_val);
        Az_Val=findViewById(R.id.Az_val);
        Gx_Val=findViewById(R.id.Gx_val);
        Gy_Val=findViewById(R.id.Gy_val);
        Gz_Val=findViewById(R.id.Gz_val);
        Mx_Val=findViewById(R.id.Mx_val);
        My_Val=findViewById(R.id.My_val);
        Mz_Val=findViewById(R.id.Mz_val);

        mlistener = new SensorEventListener() {
            @Override
            public void onAccuracyChanged(Sensor arg0, int arg1) {
            }

            @Override
            public void onSensorChanged(SensorEvent event) {
                Sensor sensor = event.sensor;
                if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

                    final double alpha =  0.8;

                    double gravity[]=new double[3];
                    double linear_acceleration[]=new double[3];
                    //Calculate Gravity
                    gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
                    gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
                    gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

                    //Remove Gravity
                    linear_acceleration[0] = event.values[0] - gravity[0];
                    linear_acceleration[1] = event.values[1] - gravity[1];
                    linear_acceleration[2] = event.values[2] - gravity[2];


                    Ax_Val.setText(String.valueOf((float)linear_acceleration[0]));
                    Ay_Val.setText(String.valueOf((float)linear_acceleration[1]));
                    Az_Val.setText(String.valueOf((float)linear_acceleration[2]));

                }
                else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                    Gx_Val.setText(String.valueOf(event.values[0]));
                    Gy_Val.setText(String.valueOf(event.values[1]));
                    Gz_Val.setText(String.valueOf(event.values[2]));
                }
                else if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
                    Mx_Val.setText(String.valueOf(event.values[0]));
                    My_Val.setText(String.valueOf(event.values[1]));
                    Mz_Val.setText(String.valueOf(event.values[2]));

                }
            }
            /////////////////////////
            // Write KALMAN Filter
           //////////////////////////////

        };
        mSensorManager.registerListener(mlistener, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mlistener, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mlistener, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_GAME);
    }
}