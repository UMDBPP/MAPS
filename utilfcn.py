########################################3
## Utility Functions
    
def cmd_mot(stepper,cmd):
    # determine direction
    if(cmd > 0):
        mot_dir = Adafruit_MotorHAT.FORWARD;
    stepper.step(1, mot_dir, Adafruit_MotorHAT.MICROSTEP)
    return 0;
    
def calc_base_attitude(mag, accel, forward = (1,0,0) , up = (0,0,1)):
    import math
    
    # calculate angle between up (base frame) and up (earth frame)
    # el_offset = acos( (accel dot up)/(norm(accel)*norm(up)) )
    el_offset = math.acos( (accel[0]*up[0]+accel[1]*up[1]+accel[2]*up[2]) / (math.sqrt(accel[0]**2+accel[1]**2+accel[2]**2)*math.sqrt(up[0]**2 + up[1]**2 + up[2]**2) ));
    el_offset = el_offset * 180/math.pi;

    # calculate angle between forward (base frame) and north (earth frame)
    # el_offset = acos( (north dot forward)/(norm(north)*norm(forward)) )
    az_offset = math.acos( (mag[0]*forward[0]+mag[1]*forward[1]+mag[2]*forward[2]) / (math.sqrt(mag[0]**2+mag[1]**2+mag[2]**2)*math.sqrt(forward[0]**2 + forward[1]**2 + forward[2]**2) ));
    az_offset = az_offset * 180/math.pi;
    
    return [az_offset, el_offset];

def ini2dict(settings_filename):
    # read in an INI file, parse it, and convert values to a dict
    import configparser
    
    f = configparser.ConfigParser()
    f.read(settings_filename)
    d = dict(f._sections)
    for k in d:
        d[k] = dict(f._defaults, **d[k])
        d[k].pop('__name__', None)

    # if the settings aren't valid, don't update what we're using and output an example file
    if not validate_settings(d):
        print('Settings did not successfully validate. Writing example settings file')
        write_ex_ini();
    else:
        d['DataIngest']['datafullpath'] = d['DataIngest']['serverip'] + '/' + d['DataIngest']['datapath'];
    return d

def validate_settings(d):
    print('Validating settings')
    expected_keys = ['Debug','DataIngest','Program','Attitude','Motors'];
    key_present = [name in d for name in expected_keys];
    missing_keys = [i for i, x in enumerate(key_present) if x == False];
    missing_sections = ', '.join([expected_keys[i] for i in missing_keys]);
    if not all(key_present):
        print('Invalid settings file. Missing sections: %s' % (missing_sections) )
        return 0;
    else:
        return 1;

def write_ex_ini():
    import configparser
    
    with open("Example_settings.ini", 'w') as fileobject:
        ex_config = configparser.RawConfigParser(allow_no_value = True)
        ex_config.add_section('Debug')
        ex_config.set('Debug', 'Debug', 'True')
        ex_config.set('Debug', '# flag which sets the program in debug mode')
        ex_config.set('Debug', '#   when in debug mode,')
        ex_config.set('Debug', '#      a json string is not retrieved from the server,')
        ex_config.set('Debug', '#       the accel, mag, and gyro are not read,')
        ex_config.set('Debug', '#       and the motors are not commanded')
        ex_config.add_section('DataIngest')
        ex_config.set('DataIngest', '# IP address of the server providing the origin and target data')
        ex_config.set('DataIngest', 'ServerIP', '192.168.1.1')
        ex_config.set('DataIngest', '# Path on the server of the json file containing the data')
        ex_config.set('DataIngest', 'DataPath', 'data.json')
        ex_config.set('DataIngest', '# Number of seconds to wait between retrieving new data')
        ex_config.set('DataIngest', 'SecBetweenFetchData', '10')
        ex_config.add_section('Program')
        ex_config.set('Program', '# Number of times per second to recalculate van attitude and command motors')
        ex_config.set('Program', 'CyclesPerSec', '1')
        ex_config.set('Program', '# Set this flag to False to end the program')
        ex_config.set('Program', 'Run', 'True')
        ex_config.add_section('Target')
        ex_config.set('Target', '# Set the callsign respresenting the location of the antenna')
        ex_config.set('Target', 'Origin_Callsign', 'W3EAX_10')
        ex_config.set('Target', '# Set the callsign of the target')
        ex_config.set('Target', 'Target_Callsign', 'W3EAX_11')
		ex_config.add_section('Attitude')
        ex_config.set('Attitude', '# vector defining upward direction in base coordinates')
        ex_config.set('Attitude', 'UpVec', '0,0,1')
        ex_config.set('Attitude', '# vector defining forward direction in base coordinates')
        ex_config.set('Attitude', 'ForwardVec', '1,0,0')
        ex_config.set('Attitude', '# data to be used as magnetometer reading during debugging')
        ex_config.set('Attitude', 'DebugMagVec', '1,0,0')
        ex_config.set('Attitude', '# data to be used as accelerometer reading during debugging')
        ex_config.set('Attitude', 'DebugAccelVec', '0,0,-1')
        ex_config.add_section('Motors')
        ex_config.set('Motors', '# port number that az motor is connected to')
        ex_config.set('Motors', 'AzMotNum', '1')
        ex_config.set('Motors', '# port number that el motor is connected to')
        ex_config.set('Motors', 'ElMotNum', '2')
        
        ex_config.write(fileobject)

    #print('Error opening file. Could not write example file');
    return 1;

def str2bool(s):
    return s.lower() in ['true','t','yes']

def setting2floattuple(s):
    return tuple([float(i) for i in s.split(',')])

def validate_json(data, settings):

    # check that json has expected structure
    if not settings['Target']['Target_Callsign'] in data and not settings['Target']['Origin_Callsign'] in data:
        print('Json failed... missing Origin and Target callsigns')
        return 0;
    
	if not 'LAT' in data[settings['Target']['Target_Callsign']] 
		print('Json failed... target definition missing LAT field')
        return 0;
		
	if not 'LNG' in data[settings['Target']['Target_Callsign']] 
		print('Json failed... target definition missing LNG field')
        return 0;
		
	if not 'ALT' in data[settings['Target']['Target_Callsign']]
		print('Json failed... target definition missing ALT field')
        return 0;
		
	if not 'TIME' in data[settings['Target']['Target_Callsign']]
		print('Json failed... target definition missing TIME field')
        return 0;
		
	if not 'LAT' in data[settings['Target']['Origin_Callsign']] 
		print('Json failed... origin definition missing LAT field')
        return 0;
		
	if not 'LNG' in data[settings['Target']['Origin_Callsign']] 
		print('Json failed... origin definition missing LNG field')
        return 0;
		
	if not 'ALT' in data[settings['Target']['Origin_Callsign']]
		print('Json failed... origin definition missing ALT field')
        return 0;
		
	if not 'TIME' in data[settings['Target']['Origin_Callsign']]
		print('Json failed... origin definition missing TIME field')
        return 0;
	
	# passed validation
    return 1;
