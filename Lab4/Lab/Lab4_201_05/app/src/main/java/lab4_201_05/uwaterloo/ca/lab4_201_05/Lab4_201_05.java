package lab4_201_05.uwaterloo.ca.lab4_201_05;

import android.graphics.PointF;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.ContextMenu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.io.File;
import java.util.Queue;
import java.util.Arrays;
import java.util.List;
import java.util.HashSet;

import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.LineSegment;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.MapView;
import java.util.LinkedList;

import java.util.ArrayList;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import lab4_201_05.uwaterloo.ca.lab4_201_05.sensortoy.LineGraphView;

import android.util.Log;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.MapLoader;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.NavigationalMap;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.PositionListener;

public class Lab4_201_05 extends AppCompatActivity  {
    TextView textfieldForDisplayDirection;
    TextView textfieldForDisplayRotationSensorValue;
    TextView textfieldForDisplayAccSensorValue;
    TextView textfieldForDisplayMagSensorValue;
    TextView textfieldForDisplaySteps;
    TextView textfieldForBs;
    TextView textfieldForCorrection;
    LinearLayout layout;
    LineGraphView graph = null;
    SensorManager sensorManager;
    Button buttonReset;
    MapView mv ;
    ImageView imgCompass;
    NavigationalMap  map;
    pl positionListener;

    PointF original;
    PointF dest;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_lab4_201_05);
        imgCompass = (ImageView) findViewById(R.id.imgCompass);
        //SET up the map
        mv = new MapView(getApplicationContext(), 1200, 1200, 40, 40);
        positionListener = new pl();
        File directory = getExternalFilesDir(null);
        Log.d("this is horrible", ""+directory);
        map = MapLoader.loadMap(getExternalFilesDir(null),"E2-3344.svg");
        mv.addListener(positionListener);
        mv.setMap(map);

        mv.setVisibility(View.VISIBLE);
        registerForContextMenu(mv);
//





        ///////////////////////////////////////////////////////
        layout = (LinearLayout) findViewById(R.id.layout);
        layout.setOrientation(LinearLayout.VERTICAL);
        graph = new LineGraphView(getApplicationContext(), 200, Arrays.asList("x", "y", "z"));

        graph.setVisibility(View.VISIBLE);
        dest =  mv.getDestinationPoint();
        original =  mv.getOriginPoint();
        positionListener.destinationChanged(mv,dest);
        positionListener.originChanged(mv,original);
        layout.addView(graph);
        layout.addView(mv, 0);

        buttonReset = (Button) findViewById(R.id.resetButton);
        textfieldForDisplayRotationSensorValue = (TextView) findViewById(R.id.rs);
        textfieldForDisplayDirection = (TextView) findViewById(R.id.ls);
        textfieldForDisplayAccSensorValue = (TextView) findViewById(R.id.as);
        textfieldForDisplayMagSensorValue = (TextView) findViewById(R.id.ms);
        textfieldForDisplaySteps = (TextView) findViewById(R.id.stepIndicator);
        textfieldForBs = (TextView) findViewById(R.id.bs);
        textfieldForCorrection = (TextView)findViewById(R.id.correct);

        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        Sensor magnetic = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor acceletor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor acceletorL = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        DirectionalPedometer ac = new DirectionalPedometer(textfieldForDisplayAccSensorValue, graph, textfieldForDisplaySteps, buttonReset,
                textfieldForDisplayRotationSensorValue,imgCompass,textfieldForBs,mv,map,original,dest,textfieldForCorrection);

        sensorManager.registerListener(ac, acceletorL, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(ac, magnetic, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(ac, acceletor, SensorManager.SENSOR_DELAY_GAME);



    }

    @Override
    public void onCreateContextMenu(ContextMenu menu, View v, ContextMenu.ContextMenuInfo menuInfo) {
        super.onCreateContextMenu(menu, v, menuInfo);
        mv.onCreateContextMenu(menu, v, menuInfo);

    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        return super.onContextItemSelected(item) || mv.onContextItemSelected(item);
    }





}
//    }
class pl implements PositionListener {
    public void originChanged(MapView source, PointF loc) {
        source.setUserPoint(loc);

    }

    public void destinationChanged(MapView source, PointF dest) {
        source.setDestinationPoint(dest);
    }


}





