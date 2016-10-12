# Summary:
#   Given coordinates of an origin and a target, calculated the view angle necessary to point at that
#   target from the origin and commands stepper motors to do so.
#
# Configuration:
#   Configurations for this software are found in 'settings.ini' in the same directory. This software
#   will adpot changes made to that file while running.
#
# Dependencies:
#   viewangle.py
#   utilfcn.py
#
# Purpose:
#   Retrieves a json file specifying the origin and target from a remote server (set via config file)
#   and calculates the motor commands necessary to point at that location taking into account the
#   attitude of the base.
#
# Author: Steve Lentine
#

#### Imports
# builtin modules
import math, json, datetime, time, os

# custom modules
import viewangle, utilfcn

# name of file to read settings from
settings_filename = "settings.ini";

#### Read configuration
settings = utilfcn.ini2dict(settings_filename);
IniCheckTime = datetime.datetime.now();

# initalizations for non-debug mode
if not settings['Debug']['debug']:
##  Reading json
    import urllib

##  Gyro
#    see http://mpolaczyk.pl/raspberry-pi-series-l3gd20-gyroscope-minimu-9-v2-python-library/
    from L3GD20 import L3GD20
    import time
     
    # Communication object
    # FIXME: update busnum with i2c address
    gyro = L3GD20(busId = 1, slaveAddr = 0x6b, ifLog = False, ifWriteBlock=False)
     
    # Configuration
    gyro.Set_PowerMode("Normal")
    gyro.Set_FullScale_Value("250dps")
    gyro.Set_AxisX_Enabled(True)
    gyro.Set_AxisY_Enabled(True)
    gyro.Set_AxisZ_Enabled(True)
    gyro.Init() # Do measurements after Init!
    gyro.Calibrate()

##  Accel/Mag
#   see https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code/blob/master/Adafruit_LSM303/Adafruit_LSM303.py
    import Adafruit_LSM303

    # FIXME: update busnum with i2c address
    accelmag = Adafruit_LSM303(busnum=-1, debug=False, hires=True)
    
##  Stepper Motors
    from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

    # create a default object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr = 0x60)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors():
            mh.getMotor[1].run(Adafruit_MotorHAT.RELEASE)
            mh.getMotor[2].run(Adafruit_MotorHAT.RELEASE)
            mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
            mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
     
    atexit.register(turnOffMotors)

    AzStepper = mh.getStepper(200, int(settings['Motors']['azmotnum']))       # 200 steps/rev, motor port #1
    ElStepper = mh.getStepper(200, int(settings['Motors']['elmotnum']))       # 200 steps/rev, motor port #2

    AzStepper.setSpeed(30)
    ElStepper.setSpeed(30)   


LastServerReadTime = 0
LastCycleTime = 0

while utilfcn.str2bool(settings['Program']['run']):

    # if settings have been updated, reload them
    if(datetime.datetime.fromtimestamp(os.stat(settings_filename).st_mtime) > IniCheckTime):
        print('Ini updated, reloading.')
        settings = utilfcn.ini2dict(settings_filename);
        IniCheckTime = datetime.datetime.now();

    # FIXME: calculate time needed to sleep to maintain timing
    time.sleep( 1/float(settings['Program']['cyclespersec']) )

    ## Read target data
    if not settings['Debug']['debug']:
    # for final implementation, read from url on pi

        if(time.clock() - LastServerReadTime > float(settings['DataIngest']['secbetweenfetchdata'])):
            data = json.load(urllib.urlopen(settings['DataIngest']['datafullpath']))
            LastServerReadTime = time.clock()
            
    else:
    # for testing, read from string
        data = json.loads('{"VAN":{"LAT":"39","LNG":"-75","ALT":"4000","STALE":"0"},"W3EAX_11":{"LAT":"39","LNG":"-76","ALT":"12000","STALE":"0"},"W3EAX_12":{"LAT":"39","LNG":"-76","ALT":"12000","STALE":"0"},"ORIGIN":"VAN","TARGET":"W3EAX_11"}');

    # check that json has expected structure
    if utilfcn.validate_json(data):
        
        # define origin and target coordinates
        origin_lla = (float(data[data['ORIGIN']]['LAT']), float(data[data['ORIGIN']]['LNG']), float(data[data['ORIGIN']]['ALT']));
        target_lla = (float(data[data['TARGET']]['LAT']), float(data[data['TARGET']]['LNG']), float(data[data['TARGET']]['ALT']));

        # calculate the viewing angles from origin to target
        [Az, El] = viewangle.ViewAngle(origin_lla,target_lla);

        print('Az ang is %f deg' % (Az));
        print('El ang is %f deg' % (El));

        if settings['Debug']['debug']:
            # test to see if result match expected
            test_tol = 0.001;

            if(abs(Az - 270.331) < test_tol and abs(El - 4.88009) < test_tol):
                print('Passed test'); 
            else:
                print('Failed Test');

    ## Read IMU data
    #   The AzEl angles calculated above are in relation to North and Up. To translate those directions into a command
    #   to send the motors, we need to know the orientation of the base of the pointing system. This is accomplished using
    #   a magnetometer to determine north and an accelerometer to determine the up directions, which are used to transform
    #   the AzEl angles into the pointing system frame

    if not settings['Debug']['debug']:
        # don't need gyro values for right now
        # dxyzn = gyro.Get_CalOut_Value()

        # [accel x, y, z, mag x, y, z]
        attitude_data = accelmag.read();
        accel = attitude_data[1:3];
        mag = attitude_data[4:6];

    else:
        accel = utilfcn.setting2floattuple(settings['Attitude']['debugaccelvec']);
        mag = utilfcn.setting2floattuple(settings['Attitude']['debugmagvec']);

    forward = utilfcn.setting2floattuple(settings['Attitude']['forwardvec']);
    up = utilfcn.setting2floattuple(settings['Attitude']['upvec']);
    
    # calculate the orientation of the base wrt North/Up
    [az_offset,el_offset] = utilfcn.calc_base_attitude(mag, accel, forward, up);
        
    az_cmd = Az + az_offset;
    el_cmd = El + el_offset;

    ## Command motors
    if not settings['Debug']['debug']:

        utilfcn.cmd_mot(AzStepper,az_cmd)
        utilfcn.cmd_mot(ElStepper,el_cmd)
        
    else:
        print('Az pointing error: %f' % (az_cmd))
        print('El pointing error: %f' % (el_cmd))


