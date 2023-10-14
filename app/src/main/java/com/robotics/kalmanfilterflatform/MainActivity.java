package com.robotics.kalmanfilterflatform;


import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.DecompositionSolver;

import java.util.Date;

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
    private TextView Roll_Val;
    private TextView Pitch_Val;
    private TextView Yaw_Val;
    private SensorEventListener mlistener;
    private SensorManager mSensorManager;
    private double theta;
    private double phi;
    private double psi;
    private double start;
    private double end;
    private Date date;
    private int[] chaged_flag={0,0,0};
    //INIT
    private double[][] x={{1},{0},{0},{0}};
    private double[][] xp={{0},{0},{0},{0}};
    private double[][] H_mat={
            {1, 0, 0,0},
            {0, 1, 0,0},
            {0, 0, 1,0},
            {0, 0, 0,1}
    };
    private double[][] P_mat={
            {1, 0, 0,0},
            {0, 1, 0,0},
            {0, 0, 1,0},
            {0, 0, 0,1}
    };
    private double[][] Q_mat={
            {0.001, 0, 0,0},
            {0, 0.001, 0,0},
            {0, 0, 0.001,0},
            {0, 0, 0,0.001}
    };

    private double[][] r_mat={
            {10, 0, 0,0},
            {0, 10, 0,0},
            {0, 0, 10,0},
            {0, 0, 0,10}
    };
    private double[][] i={
            {1, 0, 0,0},
            {0, 1, 0,0},
            {0, 0, 1,0},
            {0, 0, 0,1}
    };
    private double[][] A_mat={
            {1, 0, 0,0},
            {0, 1, 0,0},
            {0, 0, 1,0},
            {0, 0, 0,1}
    };
    private RealMatrix Z=MatrixUtils.createRealMatrix(x);
    private RealMatrix X=MatrixUtils.createRealMatrix(x);
    private RealMatrix Xp=MatrixUtils.createRealMatrix(xp);
    private RealMatrix H=MatrixUtils.createRealMatrix(H_mat);
    private RealMatrix P=MatrixUtils.createRealMatrix(P_mat);
    private RealMatrix Q=MatrixUtils.createRealMatrix(Q_mat);
    private RealMatrix R_mat=MatrixUtils.createRealMatrix(r_mat);
    private RealMatrix A=MatrixUtils.createRealMatrix(A_mat);
    private RealMatrix I=MatrixUtils.createRealMatrix(i);
    private RealMatrix Pp=MatrixUtils.createRealMatrix(i);
    private RealMatrix K=MatrixUtils.createRealMatrix(i);
    private double[] KF_res={0,0,0};
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        date=new Date();
        Ax_Val=findViewById(R.id.Ax_val);
        Ay_Val=findViewById(R.id.Ay_val);
        Az_Val=findViewById(R.id.Az_val);
        Gx_Val=findViewById(R.id.Gx_val);
        Gy_Val=findViewById(R.id.Gy_val);
        Gz_Val=findViewById(R.id.Gz_val);
        Mx_Val=findViewById(R.id.Mx_val);
        My_Val=findViewById(R.id.My_val);
        Mz_Val=findViewById(R.id.Mz_val);
        Roll_Val=findViewById(R.id.Roll_val);
        Pitch_Val=findViewById(R.id.Pitch_val);
        Yaw_Val=findViewById(R.id.Yaw_val);
        start= date.getTime();
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

//                    //Remove Gravity
//                    linear_acceleration[0] = event.values[0] - gravity[0];
//                    linear_acceleration[1] = event.values[1] - gravity[1];
//                    linear_acceleration[2] = event.values[2] - gravity[2];


                    Ax_Val.setText(String.valueOf((float)gravity[0]));
                    Ay_Val.setText(String.valueOf((float)gravity[1]));
                    Az_Val.setText(String.valueOf((float)gravity[2]));
                    accel2euler(gravity[0],gravity[1],gravity[2]);
                    euler2quater(phi,theta,0);

                }
                else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                    double p=event.values[0];
                    double q=event.values[1];
                    double r=event.values[2];
                    Gx_Val.setText(String.valueOf(event.values[0]));
                    Gy_Val.setText(String.valueOf(event.values[1]));
                    Gz_Val.setText(String.valueOf(event.values[2]));

                    end=date.getTime();
                    double dt=end-start;
                    double[][] F_mat={
                            {0, -p, -q,-r},
                            {p, 0, r,-q},
                            {q, -r, 0,p},
                            {r, q, -p,0}
                    };
                    RealMatrix F=MatrixUtils.createRealMatrix(F_mat);
                    A=I.add(F.scalarMultiply(0.5*dt));
                    KF();
                    Roll_Val.setText(String.valueOf(KF_res[0]));
                    Pitch_Val.setText(String.valueOf(KF_res[1]));
                    Yaw_Val.setText(String.valueOf(KF_res[2]));
                    start=end;
                }


                else if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
                    Mx_Val.setText(String.valueOf(event.values[0]));
                    My_Val.setText(String.valueOf(event.values[1]));
                    Mz_Val.setText(String.valueOf(event.values[2]));


                }

            }
        };
        mSensorManager.registerListener(mlistener, mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mlistener, mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mlistener, mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_GAME);
    }
    /////////////////////////
    // Write KALMAN Filter
    //////////////////////////////
    public void accel2euler(double ax, double ay, double az){
        double g= 9.81;
        theta= Math.asin(ax/g);
        phi=Math.asin(-ay/(g*Math.cos(theta)));
    }
    public void euler2quater(double roll,double pitch,double yaw){
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);

        Z.setEntry(0,0,cr * cp * cy + sr * sp * sy);
        Z.setEntry(1,0,cr * cp * cy + sr * sp * sy);
        Z.setEntry(2,0,cr * cp * cy + sr * sp * sy);
        Z.setEntry(3,0,cr * cp * cy + sr * sp * sy);

    }
    public void KF(){
        Xp=A.multiply(X);
        Pp=Q.add(A.multiply(P.multiply(A.transpose())));
        K=Pp.multiply(H.transpose().multiply(inv(R_mat.add(H.multiply(Pp.multiply(H.transpose()))))));
        X=Xp.add(K.multiply(Z.subtract(H.multiply(Xp))));
        P=Pp.subtract(K.multiply(H.multiply(Pp)));
        KF_res[0] = Math.atan2(2*(X.getEntry(2,0)*X.getEntry(3,0) + X.getEntry(0,0)*X.getEntry(1,0)), 1 - 2*(Math.pow(X.getEntry(1,0),2) + Math.pow(X.getEntry(2,0),2)));
        KF_res[1]  = -Math.asin(2*(X.getEntry(1,0)*X.getEntry(3,0) - X.getEntry(0,0)*X.getEntry(2,0)));
        KF_res[2]  =Math.atan2(2*(X.getEntry(1,0)*X.getEntry(2,0) + X.getEntry(0,0)*X.getEntry(3,0)), 1 - 2*(Math.pow(X.getEntry(2,0),2) + Math.pow(X.getEntry(3,0),2)));

    }
    public RealMatrix inv(RealMatrix matrix){
        LUDecomposition decomposition = new LUDecomposition(matrix);
        DecompositionSolver solver = decomposition.getSolver();
        if (solver.isNonSingular()) {
            RealMatrix inverse = solver.getInverse();
            return inverse;
        } else {
            System.out.println("Matrix is singular, inverse cannot be calculated.");
            return I;
        }
    }
}