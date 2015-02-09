package com.orbotix.drivesample;

import android.app.Activity;
import android.app.FragmentTransaction;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;

import com.github.mikephil.charting.charts.LineChart;
import com.github.omegabyte.drivesample.PaintView;
import com.orbotix.ConvenienceRobot;
import com.orbotix.Ollie;
import com.orbotix.SensorControl;
import com.orbotix.Sphero;
import com.orbotix.async.DeviceSensorAsyncMessage;
import com.orbotix.calibration.api.CalibrationEventListener;
import com.orbotix.calibration.api.CalibrationImageButtonView;
import com.orbotix.calibration.api.CalibrationView;
import com.orbotix.classic.DiscoveryAgentClassic;
import com.orbotix.classic.RobotClassic;
import com.orbotix.colorpicker.api.ColorPickerEventListener;
import com.orbotix.colorpicker.api.ColorPickerFragment;
import com.orbotix.command.RawMotorCommand;
import com.orbotix.common.*;
import com.orbotix.common.internal.AsyncMessage;
import com.orbotix.common.internal.DeviceResponse;
import com.orbotix.common.sensor.DeviceSensorsData;
import com.orbotix.common.sensor.LocatorSensor;
import com.orbotix.common.sensor.SensorFlag;
import com.orbotix.joystick.api.JoystickEventListener;
import com.orbotix.joystick.api.JoystickView;
import com.orbotix.le.DiscoveryAgentLE;
import com.orbotix.le.RobotLE;
import com.orbotix.robot_picker.RobotPickerDialog;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends Activity implements RobotPickerDialog.RobotPickerListener,
                                                      DiscoveryAgentEventListener,
                                                      RobotChangedStateListener, ResponseListener{

    private static final String TAG = "MainActivity";
    private static final int DEFAULT_SPIN_SPEED = 80;
    private static final int MINIMUM_MOTOR_SPEED = 40;
    private static final int MAXIMUM_MOTOR_SPEED = 250;
    private static final int MAXIMUM_GRAPH_LENGTH = 80;


    /**
     * Our current discovery agent that we will use to find robots of a certain protocol
     */
    private DiscoveryAgent _currentDiscoveryAgent;

    /**
     * The dialog that will allow the user to chose which type of robot to connect to
     */
    private RobotPickerDialog _robotPickerDialog;

    /**
     * The joystick that we will use to send roll commands to the robot
     */
    private JoystickView _joystick;

    /**
     * The connected robot
     */
    private ConvenienceRobot _connectedRobot;

    /**
     * The calibration view, used for setting the default heading of the robot
     */
    private CalibrationView _calibrationView;

    /**
     * A button used for one finger calibration
     */
    private CalibrationImageButtonView _calibrationButtonView;

    /**
     * The fragment to show that contains the color picker
     */
    private ColorPickerFragment _colorPicker;

    /**
     * The button used to bring up the color picker
     */
    private Button _colorPickerButton;

    /**
     * The button to set spin mode
     */
    private Switch _spinSwitch;
    /**
     * The button to set developer mode
     */
    private Switch _developerModeSwitch;

    /**
     * Reference to the layout containing the developer mode switch and label
     */
    private LinearLayout _developerModeLayout;

    private SeekBar _speedSeek;

    private LineChart _graph;

    private TextView _RSSI_value;
    private TextView _RSSI_median;
    private TextView _gyro;
    private TextView _accel;
    private TextView _location;
    private Button _orientButton;

    private LocatorSensor mNormalizedLocation;
    private LocatorSensor mLastLocation;
    private RSSI_Service mRSSIservice;
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName,
                                       IBinder service) {

            Log.d(TAG, "Attempting to initialize RSSI service.");
            mRSSIservice = ((RSSI_Service.LocalBinder) service)
                    .getService();
            if (!mRSSIservice.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            Log.d(TAG, "Bluetooth Service Disconnected");
            mRSSIservice = null;
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);


        setupGraph();
        setupLocatorButton();
        setupOrientationButton();
        setupSpinButton();
        setupSpeedSeeker();
        setupJoystick();
        setupCalibration();
        setupColorPicker();
        mNormalizedLocation = updateLocation(new LocatorSensor(),0,0);
        mLastLocation = updateLocation(new LocatorSensor(),0,0);

        Log.d(TAG,"Creating Intent");
        Intent gattServiceIntent = new Intent(this, RSSI_Service.class);
        Log.d(TAG,"Binding Service");
        if (bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE))
            Log.d(TAG,"Binding task finished.");
        else
            Log.d(TAG, "Binding unsuccessful.");


        // Here, you need to route all the touch events to the joystick and calibration view so that they know about
        // them. To do this, you need a way to reference the view (in this case, the id "entire_view") and attach
        // an onTouchListener which in this case is declared anonymously and invokes the
        // Controller#interpretMotionEvent() method on the joystick and the calibration view.
        findViewById(R.id.entire_view).setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                _joystick.interpretMotionEvent(event);
                _calibrationView.interpretMotionEvent(event);
                return true;
            }
        });

        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        //mRSSIService = new RSSI_Service();



        setupLocalizerButton();
    }

    private void spinRobot(ConvenienceRobot robot, int speed) {
        if(speed > MINIMUM_MOTOR_SPEED) {
            robot.setRawMotors(
                    RawMotorCommand.MotorMode.MOTOR_MODE_FORWARD,
                    speed,
                    RawMotorCommand.MotorMode.MOTOR_MODE_REVERSE,
                    speed);
        }else if (speed < MINIMUM_MOTOR_SPEED){
            robot.setRawMotors(
                    RawMotorCommand.MotorMode.MOTOR_MODE_REVERSE,
                    speed,
                    RawMotorCommand.MotorMode.MOTOR_MODE_FORWARD,
                    speed);
        }
        else{
            //stop?
            robot.stop();
            robot.setRawMotors(
                    RawMotorCommand.MotorMode.MOTOR_MODE_BRAKE,
                    0,
                    RawMotorCommand.MotorMode.MOTOR_MODE_BRAKE,
                    0);
        }
    }

    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();

        intentFilter.addAction(RSSI_Service.ACTION_GATT_CONNECTED);
        intentFilter.addAction(RSSI_Service.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(RSSI_Service.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(RSSI_Service.ACTION_DATA_AVAILABLE);
        intentFilter.addAction(RSSI_Service.ACTION_GATT_RSSI);

        return intentFilter;
    }

    @Override
    protected void onStart() {
        super.onStart();
        // Create a robot picker dialog, this allows the user to select which robot they would like to connect to.
        // We don't need to do this step if we know which robot we want to talk to, and don't need the user to
        // decide that.
        if (_robotPickerDialog == null) {
            _robotPickerDialog = new RobotPickerDialog(this, this);
        }
        // Show the picker only if it's not showing. This keeps multiple calls to onStart from showing too many pickers.
        if (!_robotPickerDialog.isShowing()) {
            _robotPickerDialog.show();
        }
/*
        _currentDiscoveryAgent = DiscoveryAgentLE.getInstance();
        // Now that we have a discovery agent, we will start discovery on it using the method defined below
        startDiscovery();
*/
    }

    @Override
    protected void onPause() {
        super.onPause();

        unregisterReceiver(mGattUpdateReceiver);
        if (mServiceConnection != null)
            unbindService(mServiceConnection);

        if (_currentDiscoveryAgent != null) {
            // When pausing, you want to make sure that you let go of the connection to the robot so that it may be
            // accessed from within other applications. Before you do that, it is a good idea to unregister for the robot
            // state change events so that you don't get the disconnection event while the application is closed.
            // This is accomplished by using DiscoveryAgent#removeRobotStateListener().
            _currentDiscoveryAgent.removeRobotStateListener(this);

            // Here we are only handling disconnecting robots if the user selected a type of robot to connect to. If you
            // didn't use the robot picker, you will need to check the appropriate discovery agent manually by using
            // DiscoveryAgent.getInstance().getConnectedRobots()
            for (Robot r : _currentDiscoveryAgent.getConnectedRobots()) {
                // There are a couple ways to disconnect a robot: sleep and disconnect. Sleep will disconnect the robot
                // in addition to putting it into standby mode. If you choose to just disconnect the robot, it will
                // use more power than if it were in standby mode. In the case of Ollie, the main LED light will also
                // turn a bright purple, indicating that it is on but disconnected. Unless you have a specific reason
                // to leave a robot on but disconnected, you should use Robot#sleep()
                r.sleep();
            }
        }
        mRSSIservice.close();
    }

    /**
     * Invoked when the user makes a selection on which robot they would like to use.
     * @param robotPicked The type of the robot that was selected
     */
    @Override
    public void onRobotPicked(RobotPickerDialog.RobotPicked robotPicked) {
        // Dismiss the robot picker so that the user doesn't keep clicking it and trying to start
        // discovery multiple times
        _robotPickerDialog.dismiss();
        switch (robotPicked) {
            // If the user picked a Sphero, you want to start the Bluetooth Classic discovery agent, as that is the
            // protocol that Sphero talks over. This will allow us to find a Sphero and connect to it.
            case Sphero:
                // To get to the classic discovery agent, you use DiscoveryAgentClassic.getInstance()
                _currentDiscoveryAgent = DiscoveryAgentClassic.getInstance();
                break;
            // If the user picked an Ollie, you want to start the Bluetooth LE discovery agent, as that is the protocol
            // that Ollie talks over. This will allow you to find an Ollie and connect to it.
            case Ollie:
                // To get to the LE discovery agent, you use DiscoveryAgentLE.getInstance()
                _currentDiscoveryAgent = DiscoveryAgentLE.getInstance();
                break;
        }

        // Now that we have a discovery agent, we will start discovery on it using the method defined below
        startDiscovery();
    }

    /**
     * Invoked when the discovery agent finds a new available robot, or updates and already available robot
     * @param robots The list of all robots, connected or not, known to the discovery agent currently
     */
    @Override
    public void handleRobotsAvailable(List<Robot> robots) {
        // Here we need to know which version of the discovery agent we are using, if we are to use Sphero, we need to
        // treat Spheros a little bit differently.
        if (_currentDiscoveryAgent instanceof DiscoveryAgentClassic) {
            // If we are using the classic discovery agent, and therefore using Sphero, we'll just connect to the first
            // one available that we get. Note that "available" in classic means paired to the phone and turned on.
            _currentDiscoveryAgent.connect(robots.get(0));
        }
        else if (_currentDiscoveryAgent instanceof DiscoveryAgentLE) {
            // If we are using the LE discovery agent, and therefore using Ollie, there's not much we need to do here.
            // The SDK will automatically connect to the robot that you touch the phone to, and you will get a message
            // saying that the robot has connected. For now, let's just log the robots that we are seeing.
            Log.v(TAG, "Robots: " + robots.toString());
            if (_robotPickerDialog.isShowing()) {
                _robotPickerDialog.dismiss();
            }
        }
    }
    /**
     * Invoked when a robot changes state. For example, when a robot connects or disconnects.
     * @param robot The robot whose state changed
     * @param type Describes what changed in the state
     */
    @Override
    public void changedState(Robot robot, RobotChangedStateNotificationType type) {
        // For the purpose of this sample, we'll only handle the connected and disconnected notifications
        switch (type) {
            // A robot was connected, and is ready for you to send commands to it.
            case Online:
                // When a robot is connected, this is a good time to stop discovery. Discovery takes a lot of system
                // resources, and if left running, will cause your app to eat the user's battery up, and may cause
                // your application to run slowly. To do this, use DiscoveryAgent#stopDiscovery().
                _currentDiscoveryAgent.stopDiscovery();
                // It is also proper form to not allow yourself to re-register for the discovery listeners, so let's
                // unregister for the available notifications here using DiscoveryAgent#removeDiscoveryListener().
                _currentDiscoveryAgent.removeDiscoveryListener(this);
                // Don't forget to turn on UI elements
                _joystick.setEnabled(true);
                _calibrationView.setEnabled(true);
                _colorPickerButton.setEnabled(true);
//                _calibrationButtonView.setEnabled(true);

                // Depending on what was connected, you might want to create a wrapper that allows you to do some
                // common functionality related to the individual robots. You can always of course use the
                // Robot#sendCommand() method, but Ollie#drive() reads a bit better.
                if (robot instanceof RobotLE) {
                    _connectedRobot = new Ollie(robot);

                    _connectedRobot.addResponseListener(this);
                    // Ollie has a developer mode that will allow a developer to poke at Bluetooth LE data manually
                    // without being disconnected. Here we set up the button to be able to enable or disable
                    // developer mode on the robot.
                    setupDeveloperModeButton();

                    Log.d(TAG,"Attempting to connect to Robot.");
                    mRSSIservice.connect(robot.getIdentifier());
                }
                else if (robot instanceof RobotClassic) {
                    _connectedRobot = new Sphero(robot);
                }

                // Finally for visual feedback let's turn the robot green saying that it's been connected
                _connectedRobot.setLed(0f, 1f, 0f);

                break;
            case Disconnected:
                // When a robot disconnects, it is a good idea to disable UI elements that send commands so that you
                // do not have to handle the user continuing to use them while the robot is not connected
                _joystick.setEnabled(false);
                _calibrationView.setEnabled(false);
                _colorPickerButton.setEnabled(false);
                //_calibrationButtonView.setEnabled(false);

                // Disable the developer mode button when the robot disconnects so that it can be set up if a LE robot
                // connectes again
                if (robot instanceof RobotLE && _developerModeLayout != null) {
                    _developerModeLayout.setVisibility(View.INVISIBLE);
                }

                // When a robot disconnects, you might want to start discovery so that you can reconnect to a robot.
                // In this case, we have no reason to not reconnect to a robot, so we will start discovery again.
                startDiscovery();
                break;
            default:
                Log.v(TAG, "Not handling state change notification: " + type);
                break;
        }
    }

    private void setupSpeedSeeker(){
        _speedSeek = (SeekBar)findViewById(R.id.speedSeek);
        _speedSeek.setMax(MAXIMUM_MOTOR_SPEED);
        _speedSeek.setProgress(DEFAULT_SPIN_SPEED);
        _speedSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {

            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                if(_spinSwitch.isChecked()){
                    spinRobot(_connectedRobot, _speedSeek.getProgress());
                }

            }
        });
    }
    private void setupOrientationButton(){
        _orientButton = (Button)findViewById(R.id.zeroHeading);
        _orientButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (_connectedRobot != null) {
                    Log.d(TAG, "Zeroing Orientation.");
                    _connectedRobot.rotate(0);//drive(0,0);
                }
            }
        });
    }
    private void setupLocatorButton(){
        _orientButton = (Button)findViewById(R.id.zeroLocation);
        _orientButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG, "Zeroing Location.");
                if(mNormalizedLocation != null && mLastLocation != null){
                    updateLocation(mNormalizedLocation,mLastLocation);
                }else{
                    Log.w(TAG,"Unable to add new normalized Location data");
                }
            }
        });
    }
    Button _localizerButton;
    PaintView paintView;
    private void setupLocalizerButton(){
        Button addRobotButton = (Button)findViewById(R.id.addRobot);
        addRobotButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG,"addRobot");
                // Create a robot picker dialog, this allows the user to select which robot they would like to connect to.
                // We don't need to do this step if we know which robot we want to talk to, and don't need the user to
                // decide that.
                // Show the picker only if it's not showing. This keeps multiple calls to onStart from showing too many pickers.
                if (_robotPickerDialog != null && !_robotPickerDialog.isShowing()) {

                    Log.d(TAG,"addRobot-show()");
                    _robotPickerDialog.show();
                }
            }
        });
        _localizerButton = (Button)findViewById(R.id.localizer);
        _localizerButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                setContentView(R.layout.multi);
                if(_setupButton == null){
                    setupSetupButton();
                }
                if(paintView == null){

                    // Getting reference to PaintView
                     paintView = (PaintView)findViewById(R.id.paint_view);

                    // Getting reference to TextView tv_cooridinate
                    //TextView tvCoordinates = (TextView)findViewById(R.id.tv_coordinates);

                    // Passing reference of textview to PaintView object to update on coordinate changes
                    //paintView.setTextView(tvCoordinates);

                    // Setting touch event listener for the PaintView
                    paintView.setOnTouchListener(paintView);
                }
            }
        });
    }
    Button _setupButton;
    private void setupSetupButton(){
        _setupButton = (Button)findViewById(R.id.setup);
        _setupButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                setContentView(R.layout.main);
                // Create a robot picker dialog, this allows the user to select which robot they would like to connect to.
                // We don't need to do this step if we know which robot we want to talk to, and don't need the user to
                // decide that.
                // Show the picker only if it's not showing. This keeps multiple calls to onStart from showing too many pickers.
                /*if (!_robotPickerDialog.isShowing()) {
                    _robotPickerDialog.show();
                }*/
            }
        });
    }
    private void setupSpinButton() {

        _spinSwitch = (Switch)findViewById(R.id.spinSwitch);

        _spinSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(_connectedRobot == null){
                    _spinSwitch.setChecked(false);
                    return;
                }
                if (isChecked){
                    Log.i(TAG,"Commanding robot to spin.");//spin
                    _joystick.setEnabled(false);
                    spinRobot(_connectedRobot, _speedSeek.getProgress());
                    mRSSIservice.clearMedian();
                    mRSSIservice.collectMedian(true);

                }
                else{
                    _joystick.setEnabled(true);
                    Log.i(TAG, "Commanding robot to stop spinning.");//stop spin
                    _connectedRobot.stop();
                    mRSSIservice.collectMedian(false);
                    //_connectedRobot.drive(0,0);
                }
            }
        });
    }

    //LineDataSet _dataRSSIraw;
    //LineData _dataGraph;

    private void setupGraph(){
        _RSSI_value = (TextView) findViewById(R.id.rssi_val);
        _RSSI_median = (TextView) findViewById(R.id.rssi_median);
        _location = (TextView) findViewById(R.id.locator);
        _accel = (TextView) findViewById(R.id.accelerometer);
        _gyro = (TextView) findViewById(R.id.gyrometer);
        //_graph = (LineChart) findViewById(R.id.graph);
        /*
        ArrayList<Entry> entries = new ArrayList<>();

        _dataRSSIraw = new LineDataSet( entries , "RSSI data");

        // set the line to be drawn like this "- - - - - -"
        _dataRSSIraw.enableDashedLine(10f, 5f, 0f);
        _dataRSSIraw.setColor(Color.BLACK);
        _dataRSSIraw.setCircleColor(Color.BLACK);
        _dataRSSIraw.setLineWidth(1f);
        _dataRSSIraw.setCircleSize(4f);
        _dataRSSIraw.setFillAlpha(65);
        _dataRSSIraw.setFillColor(Color.BLACK);

        ArrayList<LineDataSet> dataSets = new ArrayList<LineDataSet>();
        dataSets.add(_dataRSSIraw); // add the datasets

        ArrayList<String> xVals = new ArrayList<String>();
        for (int i = 0; i < MAXIMUM_GRAPH_LENGTH; i++) {
            xVals.add((i) + "");
        }

        // create a data object with the datasets
        _dataGraph = new LineData(xVals, dataSets);

        //_graph.setData(_dataGraph);
        */
        /*setData(MAXIMUM_GRAPH_LENGTH, 100);
        _dataGraph = _graph.getData();
        _dataRSSIraw = _dataGraph.getDataSetByLabel("rssi_data", false);
        if(_dataRSSIraw == null){
            Log.w(TAG,"NULL DATA SET");
        }
        _graph.invalidate();*/

    }
    //todo: remove set

    /*private void setData(int count, float range) {

        ArrayList<String> xVals = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            xVals.add((i) + "");
        }

        ArrayList<Entry> yVals = new ArrayList<>();

        for (int i = 0; i < count; i++) {
            float mult = (range + 1);
            float val = (float) (Math.random() * mult) + 3;// + (float)
            // ((mult *
            // 0.1) / 10);
            yVals.add(new Entry(val, i));
        }

        // create a dataset and give it a type
        LineDataSet set1 = new LineDataSet(yVals, "rssi_data");
        // set1.setFillAlpha(110);
        // set1.setFillColor(Color.RED);

        // set the line to be drawn like this "- - - - - -"
        set1.enableDashedLine(10f, 5f, 0f);
        set1.setColor(Color.BLACK);
        set1.setCircleColor(Color.BLACK);
        set1.setLineWidth(1f);
        set1.setCircleSize(4f);
        set1.setFillAlpha(65);
        set1.setFillColor(Color.BLACK);
        // set1.setShader(new LinearGradient(0, 0, 0, mChart.getHeight(),
        // Color.BLACK, Color.WHITE, Shader.TileMode.MIRROR));

        ArrayList<LineDataSet> dataSets = new ArrayList<LineDataSet>();
        dataSets.add(set1); // add the datasets

        // create a data object with the datasets
        LineData data = new LineData(xVals, dataSets);

        LimitLine ll1 = new LimitLine(130f);
        ll1.setLineWidth(4f);
        ll1.enableDashedLine(10f, 10f, 0f);
        ll1.setDrawValue(true);
        ll1.setLabelPosition(LimitLine.LimitLabelPosition.RIGHT);

        LimitLine ll2 = new LimitLine(-30f);
        ll2.setLineWidth(4f);
        ll2.enableDashedLine(10f, 10f, 0f);
        ll2.setDrawValue(true);
        ll2.setLabelPosition(LimitLine.LimitLabelPosition.RIGHT);

        data.addLimitLine(ll1);
        data.addLimitLine(ll2);

        // set data
        _graph.setData(data);
    }*/

    /**
     * Sets up the joystick from scratch
     */
    private void setupJoystick() {
        // Get a reference to the joystick view so that we can use it to send roll commands
        _joystick = (JoystickView)findViewById(R.id.joystickView);
        // In order to get the events from the joystick, you need to implement the JoystickEventListener interface
        // (or declare it anonymously) and set the listener.
        _joystick.setJoystickEventListener(new JoystickEventListener() {
            /**
             * Invoked when the user starts touching on the joystick
             */
            @Override
            public void onJoystickBegan() {
                // Here you can do something when the user starts using the joystick.
            }

            /**
             * Invoked when the user moves their finger on the joystick
             * @param distanceFromCenter The distance from the center of the joystick that the user is touching from 0.0 to 1.0
             *                           where 0.0 is the exact center, and 1.0 is the very edge of the outer ring.
             * @param angle The angle from the top of the joystick that the user is touching.
             */
            @Override
            public void onJoystickMoved(double distanceFromCenter, double angle) {
                // Here you can use the joystick input to drive the connected robot. You can easily do this with the
                // ConvenienceRobot#drive() method
                // Note that the arguments do flip here from the order of parameters
                Log.d(TAG,String.format("Driving at %.2f with speed %d",angle,_speedSeek.getProgress()));
                _connectedRobot.drive((float)angle, (float)_speedSeek.getProgress());//float)distanceFromCenter);
                //mRSSIservice.getDevices();
                //Log.d(TAG,_connectedRobot.getRobot().toString());
                /*if (_connectedRobot.getRobot() instanceof RobotLE) {
                    RobotLE robotLE = (RobotLE) _connectedRobot.getRobot();
                    Log.i(TAG,String.format("ROBOT_INFO: Name: %s - ID: %s - Signal Quality: %f",robotLE.getName(),robotLE.getIdentifier(),robotLE.getSignalQuality()));
                }*/
            }

            /**
             * Invoked when the user stops touching the joystick
             */
            @Override
            public void onJoystickEnded() {
                // Here you can do something when the user stops touching the joystick. For example, we'll make it stop driving.
                _connectedRobot.stop();
            }
        });

        // It is also a good idea to disable the joystick when a robot is not connected so that you do not have to
        // handle the user using the joystick while there is no robot connected.
        _joystick.setEnabled(false);
    }

    /**
     * Sets up the calibration gesture and button
     */
    private void setupCalibration() {
        // Get the view from the xml file
        _calibrationView = (CalibrationView)findViewById(R.id.calibrationView);
        // Set the glow. You might want to not turn this on if you're using any intense graphical elements.
        _calibrationView.setShowGlow(true);
        // Register anonymously for the calibration events here. You could also have this class implement the interface
        // manually if you plan to do more with the callbacks.
        _calibrationView.setCalibrationEventListener(new CalibrationEventListener() {
            /**
             * Invoked when the user begins the calibration process.
             */
            @Override
            public void onCalibrationBegan() {
                // The easy way to set up the robot for calibration is to use ConvenienceRobot#calibrating(true)
                Log.v(TAG, "Calibration began!");
                _connectedRobot.calibrating(true);
            }

            /**
             * Invoked when the user moves the calibration ring
             * @param angle The angle that the robot has rotated to.
             */
            @Override
            public void onCalibrationChanged(float angle) {
                // The usual thing to do when calibration happens is to send a roll command with this new angle, a speed of 0
                // and the calibrate flag set.
                _connectedRobot.rotate(angle);
            }

            /**
             * Invoked when the user stops the calibration process
             */
            @Override
            public void onCalibrationEnded() {
                // This is where the calibration process is "committed". Here you want to tell the robot to stop as well as
                // stop the calibration process.
                _connectedRobot.stop();
                _connectedRobot.calibrating(false);
            }
        });
        // Like the joystick, turn this off until a robot connects.
        _calibrationView.setEnabled(false);

        // To set up the button, you need a calibration view. You get the button view, and then set it to the
        // calibration view that we just configured.
       /* _calibrationButtonView = (CalibrationImageButtonView) findViewById(R.id.calibrateButton);
        _calibrationButtonView.setCalibrationView(_calibrationView);
        _calibrationButtonView.setEnabled(false);*/
    }

    /**
     * Sets up a new color picker fragment from scratch
     */
    private void setupColorPicker() {
        // To start, make a color picker fragment
        _colorPicker = new ColorPickerFragment();
        // Make sure you register for the change events. You will want to send the result of the picker to the robot.
        _colorPicker.setColorPickerEventListener(new ColorPickerEventListener() {

            /**
             * Called when the user changes the color picker
             * @param red The selected red component
             * @param green The selected green component
             * @param blue The selected blue component
             */
            @Override
            public void onColorPickerChanged(int red, int green, int blue) {
                Log.v(TAG, String.format("%d, %d, %d", red, green, blue));
                _connectedRobot.setLed(red, green, blue);
            }
        });

        // Find the color picker fragment and add a click listener to show the color picker
        _colorPickerButton = (Button)findViewById(R.id.colorPickerButton);
        _colorPickerButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                FragmentTransaction transaction = getFragmentManager().beginTransaction();
                transaction.add(R.id.fragment_root, _colorPicker, "ColorPicker");
                transaction.show(_colorPicker);
                transaction.addToBackStack("DriveSample");
                transaction.commit();
            }
        });

    }

    private void setupDeveloperModeButton() {
        // Getting the developer mode button
        if (_developerModeLayout == null)
        {
            _developerModeSwitch = (Switch)findViewById(R.id.developerModeSwitch);
            _developerModeLayout = (LinearLayout)findViewById(R.id.developerModeLayout);

            _developerModeSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
                @Override
                public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                    // We need to get the raw robot, as setting developer mode is an advanced function, and is not
                    // available on the Ollie object.
                    RobotLE robotLE = (RobotLE) _connectedRobot.getRobot();
                    robotLE.setDeveloperMode(isChecked);
                    if(isChecked){
                        _connectedRobot.enableCollisions(true);
                        //_connectedRobot.enableLocator(true);
                        _connectedRobot.enableSensors(
                                SensorFlag.GYRO_NORMALIZED.longValue()
                                        | SensorFlag.ACCELEROMETER_NORMALIZED.longValue()
                                        | SensorFlag.LOCATOR.longValue(),
                                SensorControl.StreamingRate.STREAMING_RATE100);
                        //setData(45, 100);
                        //_graph.invalidate();
                    }
                    else{
                        _connectedRobot.disableSensors();
                        _connectedRobot.enableLocator(false);
                        _connectedRobot.enableCollisions(false);
                        //_graph.setData(_dataGraph);
                        //_graph.invalidate();
                    }
                }
            });
        }
        _developerModeLayout.setVisibility(View.VISIBLE);
    }


    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (RSSI_Service.ACTION_GATT_DISCONNECTED.equals(action)) {
                Log.d(TAG,"GATT Disconnected");
            } else if (RSSI_Service.ACTION_GATT_SERVICES_DISCOVERED
                    .equals(action)) {

                Log.d(TAG,"GATT Services Discovered");
                mRSSIservice.startReadRSSI(RSSI_Service.RSSI_RATE.FIVE_HZ);
                //getGattService(mRSSIservice.getSupportedGattService());
            } else if (RSSI_Service.ACTION_DATA_AVAILABLE.equals(action)) {
                Log.d(TAG,"DATA available");
                //data = intent.getByteArrayExtra(RSSI_Service.EXTRA_DATA);
                //parseData(data);
                //readAnalogInValue(data);
            /*}else if (RSSI_Service.ACTION_GATT_RSSI_MEDIAN.equals(action)) {
                String median = intent.getStringExtra(RSSI_Service.RSSI_DATA_MEDIAN);
                _RSSI_median.setText(median);
            */
            }else if (RSSI_Service.ACTION_GATT_RSSI.equals(action)) {
                Integer rssi = intent.getIntExtra(RSSI_Service.RSSI_DATA, 1);
                Integer median = intent.getIntExtra(RSSI_Service.RSSI_DATA_MEDIAN, 1);
                if (median < 0){
                    Log.v(TAG, String.format("RSSI: %d => %s", rssi, median));
                    _RSSI_median.setText(median.toString());
                }
                if (rssi >= 0){
                    _RSSI_value.setText(" N/A ");
                    Log.w(TAG,String.format("RSSI >= 0: (%d)",rssi));
                }
                else {
                    _RSSI_value.setText(rssi.toString());

                    /*
                    int entryCount = _dataRSSIraw.getEntryCount();
                    Log.d(TAG, String.format("1Graph has %d / %d entries (%d)", entryCount, MAXIMUM_GRAPH_LENGTH, _dataGraph.getXValCount()));
                    boolean rem = false;
                    if(entryCount >= MAXIMUM_GRAPH_LENGTH)
                        rem = _dataRSSIraw.removeEntry(_dataRSSIraw.getEntryForXIndex(1));
                    _dataRSSIraw.addEntry(new Entry(rssi, 0));
                    _dataRSSIraw.notifyDataSetChanged();
                    entryCount = _dataRSSIraw.getEntryCount();
                    Log.d(TAG, String.format("Graph(%d) %s.", entryCount, (rem? "removed":"updated") ));
                    Log.d(TAG, String.format("2Graph has %d / %d entries (%d)", entryCount, MAXIMUM_GRAPH_LENGTH, _dataGraph.getXValCount()));
                    _dataGraph.notifyDataChanged();
                    _graph.setData(_dataGraph);
                    _graph.invalidate();
                    Log.d(TAG, String.format("Graph(%d) %s.", entryCount, (rem? "removed":"updated") ));
                    */

                }
            }
        }


    };

    /**
     * Starts discovery on the set discovery agent and look for robots
     */
    private void startDiscovery() {
        try {
            // You first need to set up so that the discovery agent will notify you when it finds robots.
            // To do this, you need to implement the DiscoveryAgentEventListener interface (or declare
            // it anonymously) and then register it on the discovery agent with DiscoveryAgent#addDiscoveryListener()
            _currentDiscoveryAgent.addDiscoveryListener(this);
            // Second, you need to make sure that you are notified when a robot changes state. To do this,
            // implement RobotChangedStateListener (or declare it anonymously) and use
            // DiscoveryAgent#addRobotStateListener()
            _currentDiscoveryAgent.addRobotStateListener(this);
            // Then to start looking for a Sphero, you use DiscoveryAgent#startDiscovery()
            // You do need to handle the discovery exception. This can occur in cases where the user has
            // Bluetooth off, or when the discovery cannot be started for some other reason.
            _currentDiscoveryAgent.startDiscovery(this);
        } catch (DiscoveryException e) {
            Log.e(TAG, "Could not start discovery. Reason: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void handleResponse(DeviceResponse deviceResponse, Robot robot) {

    }

    @Override
    public void handleStringResponse(String s, Robot robot) {

    }

    private LocatorSensor normalizeLocation(LocatorSensor current, LocatorSensor storedLocation){
        current.x=current.x-storedLocation.x;
        current.y=current.y-storedLocation.y;
        return current;
    }

    private LocatorSensor updateLocation(LocatorSensor location, float x, float y){
        location.x = x;
        location.y = y;
        return location;
    }
    private LocatorSensor updateLocation(LocatorSensor location, LocatorSensor newLocation){
        location.x = newLocation.x;
        location.y = newLocation.y;
        return location;
    }

    @Override
    public void handleAsyncMessage(AsyncMessage asyncMessage, Robot robot) {
        Log.d(TAG,"Async found:");
        if(asyncMessage instanceof DeviceSensorAsyncMessage){
            //Log.d(TAG,"SensorAsync found:");
            //only seems to be one data
            //ArrayList<DeviceSensorsData> datas = ((DeviceSensorAsyncMessage) asyncMessage).getAsyncData();
            //for (DeviceSensorsData data : datas){

            ArrayList<DeviceSensorsData> asyncData = ((DeviceSensorAsyncMessage) asyncMessage).getAsyncData();
            if(asyncData != null) {
                DeviceSensorsData data = asyncData.get(0);
                if(data != null) {
                    LocatorSensor location = data.getLocatorData().getPosition();
                    if (location != null){
                        if(mNormalizedLocation != null){
                            updateLocation(mLastLocation,location);
                            normalizeLocation(location,mNormalizedLocation);
                        }
                        else{
                            Log.w(TAG,"Unable to normalize Location data.");
                        }
                        _location.setText(location.toString());
                    }
                    else{
                        _location.setText(data.getLocatorData().toString());
                    }
                    _gyro.setText(data.getGyroData().toString());
                    _accel.setText(data.getAccelerometerData().toString());
                }
            }
            /*Log.d(TAG,String.format("SensorData found(%d): [%s %s %s]",
                    location.getTimeStamp(),
                    location.toString(),
                    gyro.toString(),
                    accel.toString()
            ));*/

        }
    }
}
