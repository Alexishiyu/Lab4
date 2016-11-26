package lab4_201_05.uwaterloo.ca.lab4_201_05;


/**
 * Created by alexiszhang on 2016-05-17.
 */
import android.graphics.PointF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import lab4_201_05.uwaterloo.ca.lab4_201_05.sensortoy.LineGraphView;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.MapView;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.NavigationalMap;
import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.LineSegment;
import java.util.ArrayList;
import java.util.List;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;

import lab4_201_05.uwaterloo.ca.lab4_201_05.mapper.VectorUtils;
import java.util.Timer;

/**
 * Take in accelerometer values to display onto screen, update graph, and count the number of steps taken
 */
public class DirectionalPedometer implements SensorEventListener {
    MapView mv;
    NavigationalMap map;
    PointF currentUserPoint;
    List<PointF> keyPtsList = new ArrayList<>();
    List<PointF> userToWalkPoint;
    PointF p1 = new PointF(4, 18);
    PointF p2 = new PointF(12, 18);
    PointF p3 = new PointF(18, 18);
    PointF p4 = new PointF(20, 18);
    PointF p5 = new PointF(4, 5);
    PointF p6 = new PointF(12, 5);
    PointF p7 = new PointF(18, 5);
    PointF p8 = new PointF(20, 5);
    PointF secondPoint = new PointF();
    PointF thirdPoint = new PointF();
    PointF next;
    float userx;
    float usery;
    PointF user;
    PointF start, dest;
    List<LineSegment> neighbors;
    boolean goToP2 = false;
    boolean goToP3 = false;
    int[] azimuthArray = new int[5];
    Timer timer = new Timer();
    float distance;
    float distanceToTwo;
    float distanceToThree;
    float distanceToDest;
    float actualDistance;
    int average;
    TextView forCorrectUser;
    /*****************************************************************/
    /*************************** Constants ***************************/
    /*****************************************************************/
    //Low-pass filtering constant
    private static final int C = 25;
    //Size of ArrayList at any point in time
    private static final int LIST_SIZE = 20;

    //Possible states for a step
    private static final int STATE_IDLE = 0;
    private static final int STATE_RISING = 1;
    private static final int STATE_HIGH_PEAK = 2;
    private static final int STATE_FALLING = 3;
    private static final int STATE_LOW_PEAK = 4;
    private static final int STATE_STABILIZING = 5;

    //Threshold for leaving stateIdle
    private static final double ABOVE_NOISE_THRESH = 0.10;
    //Threshold for maximum high peak value of a step
    private static final double MAX_HIGH_THRESH = 4.00;
    //Threshold for minimum high peak value of a step
    private static final double MIN_HIGH_THRESH = 0.15;
    //Threshold for a maximum low peak absolute value of a step
    private static final double MAX_LOW_THRESH = -4.00;
    //Threshold for a minimum low peak absolute value of a steps
    private static final double MIN_LOW_THRESH = -0.15;
    //Minimum value to be considered a positive/negative slope
    private static final double MIN_SLOPE = 0.0015;

    /*****************************************************************/
    /* Appropriate Views updated by AccelerometerSensorEventListener */
    /*****************************************************************/
    private TextView lAccelDisplay;
    private TextView numStepsDisplay;
    private TextView orientationDisplay;
    private TextView directionDisplay;
    private LineGraphView graph;
    private ImageView imgCompass;

    /*****************************************************************/
    /****************** Step counter-related fields ******************/
    /*****************************************************************/
    /**
     * Linear accelerometer values
     * [0] = X-coordinate
     * [1] = Y-coordinate
     * [2] = Z-coordinate
     */
    private final float[] currentLAccelValue = new float[3];
    private final float[] previousLAccelValue = new float[3];
    private final float[] maxLAccelValue = new float[3];
    //Raw scalar number of steps
    private int rawNumSteps;
    /**
     * Vectorized number of steps taken
     * [0] = North-South axis (North = positive, South = negative)
     * [1] = West-East axis (East = positive, West = negative)
     */
    private final double[] numSteps = new double[2];
    //State for Finite State Machine (FSM)
    private int states;
    //General slope of filtered Z-coordinate accelerometer values
    private int generalSlope;
    //ArrayList containing LIST_SIZE most recent IROC slopes
    private List<Integer> slopes;

    /*****************************************************************/
    /******************** Rotation-related fields ********************/
    /*****************************************************************/
    private final float[] valuesAccelerometer = new float[3];

    private final float[] valuesMagneticField = new float[3];
    private final float[] matrixR = new float[9];
    private final float[] matrixI = new float[9];
    private final float[] matrixValues = new float[3];
    /* Rotational angles */
    private double mCurrentDegree;
    private double azimuth;
    private double pitch;
    private double roll;

    //MUST pass in appropriate Views upon creation of new AccelerometerSensorEventListener
    protected DirectionalPedometer() {
    }

    //Constructor
    public DirectionalPedometer(TextView linearAccelDisplay, LineGraphView graph, TextView viewForSteps,
                                Button resetButton, TextView orientationDisplay, ImageView imgCompass,
                                TextView directionDisplay, MapView mv, NavigationalMap map, PointF start, PointF dest,TextView forCorrectUser) {
        this.lAccelDisplay = linearAccelDisplay;
        this.graph = graph;
        this.numStepsDisplay = viewForSteps;
        this.orientationDisplay = orientationDisplay;
        this.imgCompass = imgCompass;
        this.directionDisplay = directionDisplay;
        this.forCorrectUser = forCorrectUser;
        this.rawNumSteps = 0;
        this.states = 0;
        this.slopes = new ArrayList<Integer>();

        this.mCurrentDegree = 0.0;
        this.azimuth = 0.0;
        this.pitch = 0.0;
        this.roll = 0.0;

        user = mv.getUserPoint();
//        userx = start.x;
//        usery = start.y;
        this.map = map;
        this.mv = mv;
        this.start = start;
        this.dest = dest;
        keyPtsList.add(p1);
        keyPtsList.add(p2);
        keyPtsList.add(p3);
        keyPtsList.add(p4);
        keyPtsList.add(p5);
        keyPtsList.add(p6);
        keyPtsList.add(p7);
        keyPtsList.add(p8);
//        keyPtsList.add(p9);
//        keyPtsList.add(p10);
//        keyPtsList.add(p11);
////        keyPtsList.add(p12);
//        keyPtsList.add(p13);
//        keyPtsList.add(p14);
////        keyPtsList.add(p15);
//        keyPtsList.add(p16);


        //Upon pressing the reset button, set the max accelerometer values and the number of steps taken to zero
        resetButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                for (int i = 0; i < 3; i++) {
                    previousLAccelValue[i] = 0;
                    currentLAccelValue[i] = 0;
                    maxLAccelValue[i] = 0;
                }
                rawNumSteps = 0;
                numSteps[0] = 0;
                numSteps[1] = 0;
            }
        });
    }

    //Inheritance of abstract method; currently unused
    public void onAccuracyChanged(Sensor s, int i) {
    }

    /**
     * Update appropriate filtered accelerometer values, max accelerometer values, and graph point
     *
     * @param se - Contains sensor event information (ie. sensor type, values)
     */
    public void onSensorChanged(SensorEvent se) {

        switch (se.sensor.getType()){
            case Sensor.TYPE_MAGNETIC_FIELD:
                for (int i = 0; i < 3; i++) {
                    valuesMagneticField[i] = se.values[i];
                }
                break;
            case Sensor.TYPE_ACCELEROMETER:
                for (int i = 0; i < 3; i++) {
                    valuesAccelerometer[i] = se.values[i];
                }
                calculateOrientation();
                findThePath();
                wrongDirectionWarning();
                RotateAnimation ra = new RotateAnimation((float) mCurrentDegree, -(float) azimuth , Animation.RELATIVE_TO_SELF, 0.5f, Animation.RELATIVE_TO_SELF, 0.5f);
                ra.setDuration(120);
                ra.setFillAfter(true);
                this.imgCompass.startAnimation(ra);
                mCurrentDegree = -azimuth;
                break;
            case Sensor.TYPE_LINEAR_ACCELERATION:
                //Smooth the value
                for (int i = 0; i < 3; i++) {
                    previousLAccelValue[i] = currentLAccelValue[i];
                    currentLAccelValue[i] += (se.values[i] - previousLAccelValue[i]) / C;
                }

                updateMax();

                //Collect the data of the previous LIST_SIZE amount of IROC slopes
                generalSlope = updateSlope(currentLAccelValue[1] - previousLAccelValue[1]);

                FSM();

                updateDisplayValueAndGraph();
                break;
        }

/*
        switch (se.sensor.getType()) {
            case Sensor.TYPE_MAGNETIC_FIELD:
                for (int i = 0; i < 3; i++) {
                    valuesMagneticField[i] += se.values[i];
                }

            case Sensor.TYPE_ACCELEROMETER:
                for (int i = 0; i < 3; i++) {
                    valuesAccelerometer[i] += (se.values[i] - valuesAccelerometer[i]) / 25;
                }
                calculateOrientation();
                findThePath();
                wrongDirectionWarning();

                RotateAnimation ra = new RotateAnimation((float) mCurrentDegree, -(float) azimuth, Animation.RELATIVE_TO_SELF, 0.5f, Animation.RELATIVE_TO_SELF, 0.5f);
                ra.setDuration(120);
                ra.setFillAfter(true);
                this.imgCompass.startAnimation(ra);
                mCurrentDegree = -azimuth;
                break;
            case Sensor.TYPE_LINEAR_ACCELERATION:
                //Smooth the value
                for (int i = 0; i < 3; i++) {
                    previousLAccelValue[i] = currentLAccelValue[i];
                    currentLAccelValue[i] += (se.values[i] - previousLAccelValue[i]) / C;
                }


        }


        updateMax();

        //Collect the data of the previous LIST_SIZE amount of IROC slopes
        generalSlope = updateSlope(currentLAccelValue[1] - previousLAccelValue[1]);

        FSM();

        updateDisplayValueAndGraph();
*/

    }


    /**
     * Calculate the orientation of the phone and update the azimuth, pitch, and roll values
     */
    private void calculateOrientation() {
        boolean success = SensorManager.getRotationMatrix(
                matrixR,
                matrixI,
                valuesAccelerometer,
                valuesMagneticField
        );

        if (success) {
            SensorManager.getOrientation(matrixR, matrixValues);
            azimuth = (Math.toDegrees((matrixValues[0]+360)) + 270 )% 360;
            pitch = Math.toDegrees(matrixValues[1]);
            roll = Math.toDegrees(matrixValues[2]);
        }

/*        TimerTask timer1Task = new TimerTask() {
            @Override
            public void run() {
                int numOfValue;
                for (numOfValue = 0; numOfValue < 5; numOfValue++) {
                    azimuthArray[numOfValue] = (int) azimuth;
                }
            }
        };
        timer.schedule(timer1Task, 300, 150);*/

        //azimuth = (((azimuthArray[0] + azimuthArray[1] + azimuthArray[2] + azimuthArray[3] + azimuthArray[4]) / 5) + 360) % 360;

    }

    /**
     * Update max values based on the newly filtered values
     */
    private void updateMax() {
        for (int i = 0; i < 3; i++) {
            if (maxLAccelValue[i] < Math.abs(currentLAccelValue[i])) {
                maxLAccelValue[i] = Math.abs(currentLAccelValue[i]);
            }
        }
    }

    /**
     * Update the slopes ArrayList with the newest slope value and return the majority of slopes' polarity
     *
     * @param newSlope - newest IROC slope (current Y - previous Y)
     * @return int -    [-LIST_SIZE, -ceil(LIST_SIZE/3)] for generally negative slope
     * (-ceil(LIST_SIZE/3), floor(LIST_SIZE/3)) for generally neutral slope
     * [LIST_SIZE/3, LIST_SIZE] for generally positive slope
     */
    private int updateSlope(double newSlope) {
        //Determine new slope polarity
        if (newSlope > MIN_SLOPE) {
            slopes.add(1);
        } else if (newSlope < -MIN_SLOPE) {
            slopes.add(-1);
        } else {
            slopes.add(0);
        }

        //Keep a constant List size
        if (slopes.size() > LIST_SIZE) {
            slopes.remove(0);
        }

        int slope = 0;
        for (int m : slopes) {
            slope += m;
        }

        return slope;
    }

    /**
     * Finite State Machine (FSM):
     * State Idle:
     * - Enter State Rising when Y > Maximum Noise Threshold.
     * State Rising:
     * - Return to State Idle when Y < Maximum Noise Threshold, slope is (-), OR Y > Maximum High Peak Threshold.
     * - Enter State High Peak when Y > Minimum High Peak Threshold (AND Y < Maximum High Peak Threshold).
     * State High Peak:
     * - Return to State Idle when Y > Maximum High Peak Threshold.
     * - Enter State Falling when Y < Minimum High Peak Threshold AND slope is (-).
     * State Falling:
     * - Return to State Idle when Y > Maximum High Peak Threshold, slope is (+), OR Y < Maximum Low Peak Threshold.
     * - Enter State Low Peak when Y < Minimum Low Peak Threshold (AND Y > Maximum Low Peak Threshold).
     * State Low Peak:
     * - Return to State Idle when Y < Maximum Low Peak Threshold.
     * - Enter State Stabilizing when Y > Minimum Low Peak Threshold AND slope is (+).
     * State Stabilizing:
     * - Return to State Idle when Y < Maximum Low Peak Threshold OR slope is (-).
     * - Enter State Idle and increase Step Count when Y > -(Maximum Noise Threshold).
     */
    private void FSM() {
        switch (states) {
            case STATE_IDLE:
                if (currentLAccelValue[2] > ABOVE_NOISE_THRESH) {
                    states = STATE_RISING;
                }
                break;
            case STATE_RISING:
                if (currentLAccelValue[2] < ABOVE_NOISE_THRESH || generalSlope < 0/*-(LIST_SIZE/3)*/ || currentLAccelValue[2] > MAX_HIGH_THRESH) {
                    states = STATE_IDLE;
                } else if (currentLAccelValue[2] >= MIN_HIGH_THRESH) {
                    states = STATE_HIGH_PEAK;
                }
                break;
            case STATE_HIGH_PEAK:
                if (currentLAccelValue[2] > MAX_HIGH_THRESH) {
                    states = STATE_IDLE;
                } else if (currentLAccelValue[2] <= MIN_HIGH_THRESH && generalSlope < 0/*-(LIST_SIZE/3)*/) {
                    states = STATE_FALLING;
                }
                break;
            case STATE_FALLING:
                if (currentLAccelValue[2] > MAX_HIGH_THRESH || generalSlope > 0/*(LIST_SIZE/3)*/ || currentLAccelValue[2] < MAX_LOW_THRESH) {
                    states = STATE_IDLE;
                } else if (currentLAccelValue[2] <= MIN_LOW_THRESH) {
                    states = STATE_LOW_PEAK;
                }
                break;
            case STATE_LOW_PEAK:
                if (currentLAccelValue[2] < MAX_LOW_THRESH) {
                    states = STATE_IDLE;
                } else if (currentLAccelValue[2] >= MIN_LOW_THRESH && generalSlope > 0/*(LIST_SIZE/3)*/) {
                    states = STATE_STABILIZING;
                }
                break;
            case STATE_STABILIZING:
                if (currentLAccelValue[2] < MAX_LOW_THRESH || generalSlope < 0/*-(LIST_SIZE/3)*/) {
                    states = STATE_IDLE;
                } else if (currentLAccelValue[2] >= -ABOVE_NOISE_THRESH) {
                    rawNumSteps++;
                    incStepsInDirection();
                    states = STATE_IDLE;
                }
                break;
            default:
                //Unknown state
                states = STATE_IDLE;
                break;
        }
    }

    private void incStepsInDirection() {
        userx = mv.getUserPoint().x;
        usery = mv.getUserPoint().y;

        if (135 < azimuth && azimuth < 225) {
            numSteps[0]++;
            usery++;
            mv.setUserPoint(userx,usery);
            findThePath();
            if (map.calculateIntersections(mv.getUserPoint(), secondPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), thirdPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), dest).size() != 0 ){
                usery--;
                mv.setUserPoint(userx,usery);
            }
        }

        if (315 < azimuth || azimuth < 45) {
            numSteps[0]--;
            usery--;
            mv.setUserPoint(userx,usery);
            findThePath();
            if (map.calculateIntersections(mv.getUserPoint(), secondPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), thirdPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), dest).size() != 0 ){
                usery++;
                mv.setUserPoint(userx,usery);
            }
        }

        if (45 < azimuth && azimuth < 135) {
            numSteps[1]++;
            userx++;
            mv.setUserPoint(userx,usery);
            findThePath();
            if (map.calculateIntersections(mv.getUserPoint(), secondPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), thirdPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), dest).size() != 0 ){
                userx--;
                mv.setUserPoint(userx,usery);
            }
        }

        if (225 < azimuth && azimuth < 315) {
            numSteps[1]--;
            userx--;
            mv.setUserPoint(userx,usery);
            findThePath();
            if (map.calculateIntersections(mv.getUserPoint(), secondPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), thirdPoint).size() != 0 && map.calculateIntersections(mv.getUserPoint(), dest).size() != 0 ){
                userx++;
                mv.setUserPoint(userx,usery);
            }
        }

    }

    public void updateDisplayValueAndGraph() {
        //Update screen display values
        lAccelDisplay.setText(String.format("Accel: (%.3f, %.3f, %.3f) \nMax: (%.3f, %.3f, %.3f)",
                currentLAccelValue[0],
                currentLAccelValue[1],
                currentLAccelValue[2],
                maxLAccelValue[0],
                maxLAccelValue[1],
                maxLAccelValue[2]));


        numStepsDisplay.setText("Steps: " + String.valueOf(rawNumSteps) + "\nState: " + String.valueOf(states) + "\nGeneral Slope: " + String.valueOf(generalSlope));
        orientationDisplay.setText("Azimuth: " + String.valueOf(azimuth) + "\n" + "Pitch: " + String.valueOf(pitch) + "\n" + "Roll: " + String.valueOf(roll));
        directionDisplay.setText(String.format("Steps [North]: %.2f \nSteps [East]: %.2f", numSteps[0], numSteps[1]));
        graph.addPoint(new float[]{currentLAccelValue[1], currentLAccelValue[2]});
    }

    public void findThePath() {
        userToWalkPoint = new ArrayList<PointF>();
        currentUserPoint = mv.getUserPoint();
        // setup to begin BFS
        userToWalkPoint.add(currentUserPoint);
        if (map.calculateIntersections(currentUserPoint, dest).size() == 0){
            userToWalkPoint.add(dest);
            mv.setUserPath(userToWalkPoint);
            double distancex = Math.floor((double) dest.x - (double) currentUserPoint.x);
            double distancey = (-1)*Math.floor((double) dest.y - (double) currentUserPoint.y);
            forCorrectUser.setText("Move " +distancey+ "steps North and " +distancex+ "steps East");
            return;
        }

        for (int j = 0; j < keyPtsList.size(); ++j) {
            if ((map.calculateIntersections(currentUserPoint, keyPtsList.get(j)).size() == 0)&&map.calculateIntersections(keyPtsList.get(j),dest).size() == 0) {
                secondPoint = keyPtsList.get(j);
                userToWalkPoint.add(secondPoint);
                userToWalkPoint.add(dest);
                mv.setUserPath(userToWalkPoint);
                double distancex = Math.floor((double) secondPoint.x - (double) currentUserPoint.x);
                double distancey = (-1)*Math.floor((double) secondPoint.y - (double) currentUserPoint.y);
                forCorrectUser.setText("Move " +distancey+ "steps North and " +distancex+ "steps East");
                return;
            }
            else if (map.calculateIntersections(currentUserPoint, keyPtsList.get(j)).size() == 0){
                secondPoint = keyPtsList.get(j);
                goToP2 = true;
                break;
            }
        }

        for (int j = 0; j < keyPtsList.size(); ++j) {
            if ((goToP2 &&map.calculateIntersections(secondPoint, keyPtsList.get(j)).size() == 0)&&map.calculateIntersections(keyPtsList.get(j),dest).size() == 0) {
                thirdPoint = keyPtsList.get(j);
                if (map.calculateIntersections(currentUserPoint, thirdPoint).size() == 0){
                    userToWalkPoint.add(thirdPoint);
                } else {
                    userToWalkPoint.add(secondPoint);
                    userToWalkPoint.add(thirdPoint);
                }
                userToWalkPoint.add(dest);
                mv.setUserPath(userToWalkPoint);
                double distancex = Math.floor((double) secondPoint.x - (double) currentUserPoint.x);
                double distancey = (-1)*Math.floor((double) secondPoint.y - (double) currentUserPoint.y);
                forCorrectUser.setText("Move " +distancey+ "steps North and " +distancex+ "steps East");
                return;
            }
        }

    }

    public void wrongDirectionWarning(){
        VectorUtils vU = new VectorUtils();
/*        distance = vU.distance(currentUserPoint,secondPoint)+  vU.distance(secondPoint,thirdPoint)+ vU.distance(thirdPoint,dest) ;
        actualDistance = vU.distance(startecondPoint)+  vU.distance(secondPoint,thirdPoint)+ vU.distance(thirdPoint,dest) ;
        if (actualDistance>distance){
            forCorrectUser.setText("WRONG WAY!!!!");
        }
        else forCorrectUser.setText("KEEP WALKING");*/,s
        distance = vU.distance(currentUserPoint, dest);
        if (distance < 1) {
            forCorrectUser.setText("You Have Arrived");
        }
    }
}








