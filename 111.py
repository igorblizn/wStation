import collections
import copy
import logging
import math
import struct
import time

import monotonic
from past.builtins import basestring

from pymavlink import mavutil, mavwp
from pymavlink.dialects.v10 import ardupilotmega

from dronekit.util import ErrprinterHandler


class Attitude(object):
    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
    def __str__(self):
        fmt = '{}:pitch={pitch},yaw={yaw},roll={roll}'
        return fmt.format(self.__class__.__name__, **vars(self))


class LocationGlobal(object):
    

    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None

    def __str__(self):
        return "LocationGlobal:lat=%s,lon=%s,alt=%s" % (self.lat, self.lon, self.alt)


class LocationGlobalRelative(object):
   

    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None

    def __str__(self):
        return "LocationGlobalRelative:lat=%s,lon=%s,alt=%s" % (self.lat, self.lon, self.alt)


class LocationLocal(object):
    

    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down

    def __str__(self):
        return "LocationLocal:north=%s,east=%s,down=%s" % (self.north, self.east, self.down)

    def distance_home(self):

        if self.north is not None and self.east is not None:
            if self.down is not None:
                return math.sqrt(self.north**2 + self.east**2 + self.down**2)
            else:
                return math.sqrt(self.north**2 + self.east**2)


class GPSInfo(object):
    

    def __init__(self, eph, epv, fix_type, satellites_visible):
        self.eph = eph
        self.epv = epv
        self.fix_type = fix_type
        self.satellites_visible = satellites_visible

    def __str__(self):
        return "GPSInfo:fix=%s,num_sat=%s" % (self.fix_type, self.satellites_visible)


class Battery(object):
  

    def __init__(self, voltage, current, level):
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0
        if level == -1:
            self.level = None
        else:
            self.level = level

    def __str__(self):
        return "Battery:voltage={},current={},level={}".format(self.voltage, self.current,
                                                               self.level)


class Rangefinder(object):


    def __init__(self, distance, voltage):
        self.distance = distance
        self.voltage = voltage

    def __str__(self):
        return "Rangefinder: distance={}, voltage={}".format(self.distance, self.voltage)



class Capabilities:
    
    def __init__(self, capabilities):
        self.mission_float                  = (((capabilities >> 0)  & 1) == 1)
        self.param_float                    = (((capabilities >> 1)  & 1) == 1)
        self.mission_int                    = (((capabilities >> 2)  & 1) == 1)
        self.command_int                    = (((capabilities >> 3)  & 1) == 1)
        self.param_union                    = (((capabilities >> 4)  & 1) == 1)
        self.ftp                            = (((capabilities >> 5)  & 1) == 1)
        self.set_attitude_target            = (((capabilities >> 6)  & 1) == 1)
        self.set_attitude_target_local_ned  = (((capabilities >> 7)  & 1) == 1)
        self.set_altitude_target_global_int = (((capabilities >> 8)  & 1) == 1)
        self.terrain                        = (((capabilities >> 9)  & 1) == 1)
        self.set_actuator_target            = (((capabilities >> 10) & 1) == 1)
        self.flight_termination             = (((capabilities >> 11) & 1) == 1)
        self.compass_calibration            = (((capabilities >> 12) & 1) == 1)


class VehicleMode(object):
    

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return "VehicleMode:%s" % self.name

    def __eq__(self, other):
        return self.name == other

    def __ne__(self, other):
        return self.name != other


class SystemStatus(object):
  

    def __init__(self, state):
        self.state = state

    def __str__(self):
        return "SystemStatus:%s" % self.state

    def __eq__(self, other):
        return self.state == other

    def __ne__(self, other):
        return self.state != other


class HasObservers(object):
    def __init__(self):
        logging.basicConfig()
        self._logger = logging.getLogger(__name__)

        # A mapping from attr_name to a list of observers
        self._attribute_listeners = {}
        self._attribute_cache = {}

    def add_attribute_listener(self, attr_name, observer):
        
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is None:
            listeners_for_attr = []
            self._attribute_listeners[attr_name] = listeners_for_attr
        if observer not in listeners_for_attr:
            listeners_for_attr.append(observer)

    def remove_attribute_listener(self, attr_name, observer):
       
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is not None:
            listeners_for_attr.remove(observer)
            if len(listeners_for_attr) == 0:
                del self._attribute_listeners[attr_name]

    def notify_attribute_listeners(self, attr_name, value, cache=False):
       
        # Cached values are not re-sent if they are unchanged.
        if cache:
            if self._attribute_cache.get(attr_name) == value:
                return
            self._attribute_cache[attr_name] = value

        # Notify observers.
        for fn in self._attribute_listeners.get(attr_name, []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

    def on_attribute(self, name):


        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)

        return decorator



class Locations(HasObservers):


    def __init__(self, vehicle):
        super(Locations, self).__init__()

        self._lat = None
        self._lon = None
        self._alt = None
        self._relative_alt = None

        @vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(vehicle, name, m):
            (self._lat, self._lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self._relative_alt = m.relative_alt / 1000.0
            self.notify_attribute_listeners('global_relative_frame', self.global_relative_frame)
            vehicle.notify_attribute_listeners('location.global_relative_frame',
                                               vehicle.location.global_relative_frame)

            if self._alt is not None or m.alt != 0:
                # Require first alt value to be non-0
                # TODO is this the proper check to do?
                self._alt = m.alt / 1000.0
                self.notify_attribute_listeners('global_frame', self.global_frame)
                vehicle.notify_attribute_listeners('location.global_frame',
                                                   vehicle.location.global_frame)

            vehicle.notify_attribute_listeners('location', vehicle.location)

        self._north = None
        self._east = None
        self._down = None

        @vehicle.on_message('LOCAL_POSITION_NED')
        def listener(vehicle, name, m):
            self._north = m.x
            self._east = m.y
            self._down = m.z
            self.notify_attribute_listeners('local_frame', self.local_frame)
            vehicle.notify_attribute_listeners('location.local_frame', vehicle.location.local_frame)
            vehicle.notify_attribute_listeners('location', vehicle.location)

    @property
    def local_frame(self):
       
        return LocationLocal(self._north, self._east, self._down)

    @property
    def global_frame(self):
        
        return LocationGlobal(self._lat, self._lon, self._alt)

    @property
    def global_relative_frame(self):
      
        return LocationGlobalRelative(self._lat, self._lon, self._relative_alt)


class Vehicle(HasObservers):
    

    def __init__(self, handler):
        super(Vehicle, self).__init__()

        self._logger = logging.getLogger(__name__)  # Logger for DroneKit
        self._autopilot_logger = logging.getLogger('autopilot')  # Logger for the autopilot messages
        # MAVLink-to-logging-module log severity mappings
        self._mavlink_statustext_severity = {
            0: logging.CRITICAL,
            1: logging.CRITICAL,
            2: logging.CRITICAL,
            3: logging.ERROR,
            4: logging.WARNING,
            5: logging.INFO,
            6: logging.INFO,
            7: logging.DEBUG
        }

        self._handler = handler
        self._master = handler.master

        # Cache all updated attributes for wait_ready.
        # By default, we presume all "commands" are loaded.
        self._ready_attrs = {'commands'}

        # Default parameters when calling wait_ready() or wait_ready(True).
        self._default_ready_attrs = ['parameters', 'gps_0', 'armed', 'mode', 'attitude']

        @self.on_attribute('*')
        def listener(_, name, value):
            self._ready_attrs.add(name)

        # Attaches message listeners.
        self._message_listeners = dict()

        @handler.forward_message
        def listener(_, msg):
            self.notify_message_listeners(msg.get_type(), msg)

        self._location = Locations(self)
        self._vx = None
        self._vy = None
        self._vz = None

        @self.on_message('STATUSTEXT')
        def statustext_listener(self, name, m):
            # Log the STATUSTEXT on the autopilot logger, with the correct severity
            self._autopilot_logger.log(
                msg=m.text.strip(),
                level=self._mavlink_statustext_severity[m.severity]
            )

        @self.on_message('GLOBAL_POSITION_INT')
        def listener(self, name, m):
            (self._vx, self._vy, self._vz) = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
            self.notify_attribute_listeners('velocity', self.velocity)

        self._pitch = None
        self._yaw = None
        self._roll = None
        self._pitchspeed = None
        self._yawspeed = None
        self._rollspeed = None

        @self.on_message('ATTITUDE')
        def listener(self, name, m):
            self._pitch = m.pitch
            self._yaw = m.yaw
            self._roll = m.roll
            self._pitchspeed = m.pitchspeed
            self._yawspeed = m.yawspeed
            self._rollspeed = m.rollspeed
            self.notify_attribute_listeners('attitude', self.attitude)

        self._heading = None
        self._airspeed = None
        self._groundspeed = None

        @self.on_message('VFR_HUD')
        def listener(self, name, m):
            self._heading = m.heading
            self.notify_attribute_listeners('heading', self.heading)
            self._airspeed = m.airspeed
            self.notify_attribute_listeners('airspeed', self.airspeed)
            self._groundspeed = m.groundspeed
            self.notify_attribute_listeners('groundspeed', self.groundspeed)

        self._rngfnd_distance = None
        self._rngfnd_voltage = None

        @self.on_message('RANGEFINDER')
        def listener(self, name, m):
            self._rngfnd_distance = m.distance
            self._rngfnd_voltage = m.voltage
            self.notify_attribute_listeners('rangefinder', self.rangefinder)

        self._mount_pitch = None
        self._mount_yaw = None
        self._mount_roll = None

        @self.on_message('MOUNT_STATUS')
        def listener(self, name, m):
            self._mount_pitch = m.pointing_a / 100.0
            self._mount_roll = m.pointing_b / 100.0
            self._mount_yaw = m.pointing_c / 100.0
            self.notify_attribute_listeners('mount', self.mount_status)

        self._capabilities = None
        self._raw_version = None
        self._autopilot_version_msg_count = 0

        @self.on_message('AUTOPILOT_VERSION')
        def listener(vehicle, name, m):
            self._capabilities = m.capabilities
            self._raw_version = m.flight_sw_version
            self._autopilot_version_msg_count += 1
            if self._capabilities != 0 or self._autopilot_version_msg_count > 5:
                # ArduPilot <3.4 fails to send capabilities correctly
                # straight after boot, and even older versions send
                # this back as always-0.
                vehicle.remove_message_listener('HEARTBEAT', self.send_capabilities_request)
            self.notify_attribute_listeners('autopilot_version', self._raw_version)


        # All keys are strings.
        

        @self.on_message(['RC_CHANNELS_RAW', 'RC_CHANNELS'])
        def listener(self, name, m):
            def set_rc(chnum, v):
                '''Private utility for handling rc channel messages'''
                # use port to allow ch nums greater than 8
                port = 0 if name == "RC_CHANNELS" else m.port
                self._channels._update_channel(str(port * 8 + chnum), v)

            for i in range(1, (18 if name == "RC_CHANNELS" else 8)+1):
                set_rc(i, getattr(m, "chan{}_raw".format(i)))

            self.notify_attribute_listeners('channels', self.channels)

        self._voltage = None
        self._current = None
        self._level = None

        @self.on_message('SYS_STATUS')
        def listener(self, name, m):
            self._voltage = m.voltage_battery
            self._current = m.current_battery
            self._level = m.battery_remaining
            self.notify_attribute_listeners('battery', self.battery)

        self._eph = None
        self._epv = None
        self._satellites_visible = None
        self._fix_type = None  # FIXME support multiple GPSs per vehicle - possibly by using componentId

        @self.on_message('GPS_RAW_INT')
        def listener(self, name, m):
            self._eph = m.eph
            self._epv = m.epv
            self._satellites_visible = m.satellites_visible
            self._fix_type = m.fix_type
            self.notify_attribute_listeners('gps_0', self.gps_0)

        self._current_waypoint = 0

        @self.on_message(['WAYPOINT_CURRENT', 'MISSION_CURRENT'])
        def listener(self, name, m):
            self._current_waypoint = m.seq

        self._ekf_poshorizabs = False
        self._ekf_constposmode = False
        self._ekf_predposhorizabs = False

        @self.on_message('EKF_STATUS_REPORT')
        def listener(self, name, m):
            # boolean: EKF's horizontal position (absolute) estimate is good
            self._ekf_poshorizabs = (m.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0
            # boolean: EKF is in constant position mode and does not know it's absolute or relative position
            self._ekf_constposmode = (m.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0
            # boolean: EKF's predicted horizontal position (absolute) estimate is good
            self._ekf_predposhorizabs = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0

            self.notify_attribute_listeners('ekf_ok', self.ekf_ok, cache=True)

        self._flightmode = 'AUTO'
        self._armed = False
        self._system_status = None
        self._autopilot_type = None  # PX4, ArduPilot, etc.
        self._vehicle_type = None  # quadcopter, plane, etc.

        @self.on_message('HEARTBEAT')
        def listener(self, name, m):
            # ignore groundstations
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.notify_attribute_listeners('armed', self.armed, cache=True)
            self._autopilot_type = m.autopilot
            self._vehicle_type = m.type
            
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self._flightmode = mavutil.interpret_px4_mode(m.base_mode, m.custom_mode)
            else:
                self._flightmode = self._mode_mapping_bynumber[m.custom_mode]
            self.notify_attribute_listeners('mode', self.mode, cache=True)
            self._system_status = m.system_status
            self.notify_attribute_listeners('system_status', self.system_status, cache=True)

        # Waypoints.

        self._home_location = None
        self._wploader = mavwp.MAVWPLoader()
        self._wp_loaded = True
        self._wp_uploaded = None
        self._wpts_dirty = False
        self._commands = CommandSequence(self)

        @self.on_message(['WAYPOINT_COUNT', 'MISSION_COUNT'])
        def listener(self, name, msg):
            if not self._wp_loaded:
                self._wploader.clear()
                self._wploader.expected_count = msg.count
                self._master.waypoint_request_send(0)

        @self.on_message(['HOME_POSITION'])
        def listener(self, name, msg):
            self._home_location = LocationGlobal(msg.latitude / 1.0e7, msg.longitude / 1.0e7, msg.altitude / 1000.0)
            self.notify_attribute_listeners('home_location', self.home_location, cache=True)

        @self.on_message(['WAYPOINT', 'MISSION_ITEM'])
        def listener(self, name, msg):
            if not self._wp_loaded:
                if msg.seq == 0:
                    if not (msg.x == 0 and msg.y == 0 and msg.z == 0):
                        self._home_location = LocationGlobal(msg.x, msg.y, msg.z)

                if msg.seq > self._wploader.count():
                    # Unexpected waypoint
                    pass
                elif msg.seq < self._wploader.count():
                    # Waypoint duplicate
                    pass
                else:
                    self._wploader.add(msg)

                    if msg.seq + 1 < self._wploader.expected_count:
                        self._master.waypoint_request_send(msg.seq + 1)
                    else:
                        self._wp_loaded = True
                        self.notify_attribute_listeners('commands', self.commands)

        # Waypoint send to master
        @self.on_message(['WAYPOINT_REQUEST', 'MISSION_REQUEST'])
        def listener(self, name, msg):
            if self._wp_uploaded is not None:
                wp = self._wploader.wp(msg.seq)
                handler.fix_targets(wp)
                self._master.mav.send(wp)
                self._wp_uploaded[msg.seq] = True

        # TODO: Waypoint loop listeners

        # Parameters.

        start_duration = 0.2
        repeat_duration = 1

        self._params_count = -1
        self._params_set = []
        self._params_loaded = False
        self._params_start = False
        self._params_map = {}
        self._params_last = monotonic.monotonic()  # Last new param.
        self._params_duration = start_duration
        self._parameters = Parameters(self)

        @handler.forward_loop
        def listener(_):
            # Check the time duration for last "new" params exceeds watchdog.
            if not self._params_start:
                return

            if not self._params_loaded and all(x is not None for x in self._params_set):
                self._params_loaded = True
                self.notify_attribute_listeners('parameters', self.parameters)

            if not self._params_loaded and monotonic.monotonic() - self._params_last > self._params_duration:
                c = 0
                for i, v in enumerate(self._params_set):
                    if v is None:
                        self._master.mav.param_request_read_send(0, 0, b'', i)
                        c += 1
                        if c > 50:
                            break
                self._params_duration = repeat_duration
                self._params_last = monotonic.monotonic()

        @self.on_message(['PARAM_VALUE'])
        def listener(self, name, msg):
            # If we discover a new param count, assume we
            # are receiving a new param set.
            if self._params_count != msg.param_count:
                self._params_loaded = False
                self._params_start = True
                self._params_count = msg.param_count
                self._params_set = [None] * msg.param_count

            # Attempt to set the params. We throw an error
            # if the index is out of range of the count or
            # we lack a param_id.
            try:
                if msg.param_index < msg.param_count and msg:
                    if self._params_set[msg.param_index] is None:
                        self._params_last = monotonic.monotonic()
                        self._params_duration = start_duration
                    self._params_set[msg.param_index] = msg

                self._params_map[msg.param_id] = msg.param_value
                self._parameters.notify_attribute_listeners(msg.param_id, msg.param_value,
                                                            cache=True)
            except:
                import traceback
                traceback.print_exc()

        # Heartbeats.

        self._heartbeat_started = False
        self._heartbeat_lastsent = 0
        self._heartbeat_lastreceived = 0
        self._heartbeat_timeout = False

        self._heartbeat_warning = 5
        self._heartbeat_error = 30
        self._heartbeat_system = None

        @handler.forward_loop
        def listener(_):
            # Send 1 heartbeat per second
            if monotonic.monotonic() - self._heartbeat_lastsent > 1:
                self._master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self._heartbeat_lastsent = monotonic.monotonic()

            

        @self.on_message(['HEARTBEAT'])
        def listener(self, name, msg):
            # ignore groundstations
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._heartbeat_system = msg.get_srcSystem()
            self._heartbeat_lastreceived = monotonic.monotonic()
            if self._heartbeat_timeout:
                self._logger.info('...link restored.')
            self._heartbeat_timeout = False

        self._last_heartbeat = None

        @handler.forward_loop
        def listener(_):
            if self._heartbeat_lastreceived:
                self._last_heartbeat = monotonic.monotonic() - self._heartbeat_lastreceived
                self.notify_attribute_listeners('last_heartbeat', self.last_heartbeat)

    @property
    def last_heartbeat(self):
        
        return self._last_heartbeat

    def on_message(self, name):
        

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
       
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):
        
        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        for fn in self._message_listeners.get(name, []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception('Exception in message handler for %s' % msg.get_type(), exc_info=True)

        for fn in self._message_listeners.get('*', []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception('Exception in message handler for %s' % msg.get_type(), exc_info=True)

    def close(self):
        return self._handler.close()

    def flush(self):
       
        return self.commands.upload()

    #
    # Private sugar methods
    #

    @property
    def _mode_mapping(self):
        return self._master.mode_mapping()

    @property
    def _mode_mapping_bynumber(self):
        return mavutil.mode_mapping_bynumber(self._vehicle_type)

    def _is_mode_available(self, custommode_code, basemode_code=0):
        try:
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                mode = mavutil.interpret_px4_mode(basemode_code, custommode_code)
                return mode in self._mode_mapping
            return custommode_code in self._mode_mapping_bynumber
        except:
            return False

    #
    # Operations to support the standard API.
    #

    @property
    def mode(self):
        
        if not self._flightmode:
            return None
        return VehicleMode(self._flightmode)

    @mode.setter
    def mode(self, v):
        if isinstance(v, basestring):
            v = VehicleMode(v)

        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self._master.set_mode(v.name)
        elif isinstance(v, int):
            self._master.set_mode(v)
        else:
            self._master.set_mode(self._mode_mapping[v.name])

    @property
    def location(self):
       
        return self._location

    @property
    def battery(self):
        """
        Current system batter status (:py:class:`Battery`).
        """
        if self._voltage is None or self._current is None or self._level is None:
            return None
        return Battery(self._voltage, self._current, self._level)

    @property
    def rangefinder(self):
        """
        Rangefinder distance and voltage values (:py:class:`Rangefinder`).
        """
        return Rangefinder(self._rngfnd_distance, self._rngfnd_voltage)

    @property
    def velocity(self):
        """
        Current velocity as a three element list ``[ vx, vy, vz ]`` (in meter/sec).
        """
        return [self._vx, self._vy, self._vz]

   
    @property
    def capabilities(self):
        """
        The autopilot capabilities in a :py:class:`Capabilities` object.

        .. versionadded:: 2.0.3
        """
        return Capabilities(self._capabilities)

    @property
    def attitude(self):
        """
        Current vehicle attitude - pitch, yaw, roll (:py:class:`Attitude`).
        """
        return Attitude(self._pitch, self._yaw, self._roll)

    @property
    def gps_0(self):
        """
        GPS position information (:py:class:`GPSInfo`).
        """
        return GPSInfo(self._eph, self._epv, self._fix_type, self._satellites_visible)

    @property
    def armed(self):
       
        return self._armed

    @armed.setter
    def armed(self, value):
        if bool(value) != self._armed:
            if value:
                self._master.arducopter_arm()
            else:
                self._master.arducopter_disarm()

    @property
    def is_armable(self):
        return self.mode != 'INITIALISING' and (self.gps_0.fix_type is not None and self.gps_0.fix_type > 1) and self._ekf_predposhorizabs

    @property
    def system_status(self):

        return {
            0: SystemStatus('UNINIT'),
            1: SystemStatus('BOOT'),
            2: SystemStatus('CALIBRATING'),
            3: SystemStatus('STANDBY'),
            4: SystemStatus('ACTIVE'),
            5: SystemStatus('CRITICAL'),
            6: SystemStatus('EMERGENCY'),
            7: SystemStatus('POWEROFF'),
        }.get(self._system_status, None)

    @property
    def heading(self):

        return self._heading

    @property
    def groundspeed(self):
      
        return self._groundspeed

    @groundspeed.setter
    def groundspeed(self, speed):
        speed_type = 1  # ground speed
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
            0,  # confirmation
            speed_type,  # param 1
            speed,  # speed in metres/second
            -1, 0, 0, 0, 0  # param 3 - 7
        )

        # send command to vehicle
        self.send_mavlink(msg)

    @property
    def airspeed(self):
        
        return self._airspeed

    @airspeed.setter
    def airspeed(self, speed):
        speed_type = 0  # air speed
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
            0,  # confirmation
            speed_type,  # param 1
            speed,  # speed in metres/second
            -1, 0, 0, 0, 0  # param 3 - 7
        )

        # send command to vehicle
        self.send_mavlink(msg)

    
    @property
    def mount_status(self):
        
        return [self._mount_pitch, self._mount_yaw, self._mount_roll]

    
    
    @property
    def home_location(self):
       
        return copy.copy(self._home_location)

    @home_location.setter
    def home_location(self, pos):
    

        if not isinstance(pos, LocationGlobal):
            raise ValueError('Expecting home_location to be set to a LocationGlobal.')

        # Set cached home location.
        self._home_location = copy.copy(pos)

        # Send MAVLink update.
        self.send_mavlink(self.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
            0,  # confirmation
            0,  # param 1: 1 to use current position, 0 to use the entered values.
            0, 0, 0,  # params 2-4
            pos.lat, pos.lon, pos.alt))

    @property
    def commands(self):
     
        return self._commands

    @property
    def parameters(self):
        """
        The (editable) parameters for this vehicle (:py:class:`Parameters`).
        """
        return self._parameters

    def wait_for(self, condition, timeout=None, interval=0.1, errmsg=None):
        

        t0 = time.time()
        while not condition():
            t1 = time.time()
            if timeout and (t1 - t0) >= timeout:
                raise TimeoutError(errmsg)

            time.sleep(interval)

    def wait_for_armable(self, timeout=None):
       

        def check_armable():
            return self.is_armable

        self.wait_for(check_armable, timeout=timeout)

    def arm(self, wait=True, timeout=None):
      

        self.armed = True

        if wait:
            self.wait_for(lambda: self.armed, timeout=timeout,
                          errmsg='failed to arm vehicle')

    def disarm(self, wait=True, timeout=None):
        
        self.armed = False

        if wait:
            self.wait_for(lambda: not self.armed, timeout=timeout,
                          errmsg='failed to disarm vehicle')

    def wait_for_mode(self, mode, timeout=None):
        if not isinstance(mode, VehicleMode):
            mode = VehicleMode(mode)

        self.mode = mode

        self.wait_for(lambda: self.mode.name == mode.name,
                      timeout=timeout,
                      errmsg='failed to set flight mode')

    def wait_for_alt(self, alt, epsilon=0.1, rel=True, timeout=None):
       

        def get_alt():
            if rel:
                alt = self.location.global_relative_frame.alt
            else:
                alt = self.location.global_frame.alt

            return alt

        def check_alt():
            cur = get_alt()
            delta = abs(alt - cur)

            return (
                (delta < epsilon) or
                (cur > alt > start) or
                (cur < alt < start)
            )

        start = get_alt()

        self.wait_for(
            check_alt,
            timeout=timeout,
            errmsg='failed to reach specified altitude')

    def wait_simple_takeoff(self, alt=None, epsilon=0.1, timeout=None):
        self.simple_takeoff(alt)

        if alt is not None:
            self.wait_for_alt(alt, epsilon=epsilon, timeout=timeout)

    def simple_takeoff(self, alt=None):
        
        if alt is not None:
            altitude = float(alt)
            if math.isnan(altitude) or math.isinf(altitude):
                raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
            self._master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, altitude)

    def simple_goto(self, location, airspeed=None, groundspeed=None):
        
        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            alt = location.alt
        elif isinstance(location, LocationGlobal):
            # This should be the proper code:
            # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            # However, APM discards information about the relative frame
            # and treats any alt value as relative. So we compensate here.
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            if not self.home_location:
                self.commands.download()
                self.commands.wait_ready()
            alt = location.alt - self.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        self._master.mav.mission_item_send(0, 0, 0, frame,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, location.lat, location.lon,
                                           alt)

        if airspeed is not None:
            self.airspeed = airspeed
        if groundspeed is not None:
            self.groundspeed = groundspeed

    def send_mavlink(self, message):
       
        self._master.mav.send(message)

    @property
    def message_factory(self):
       
        return self._master.mav

    def initialize(self, rate=4, heartbeat_timeout=30):
        self._handler.start()

        # Start heartbeat polling.
        start = monotonic.monotonic()
        self._heartbeat_error = heartbeat_timeout or 0
        self._heartbeat_started = True
        self._heartbeat_lastreceived = start

        # Poll for first heartbeat.
        # If heartbeat times out, this will interrupt.
        while self._handler._alive:
            time.sleep(.1)
            if self._heartbeat_lastreceived != start:
                break
        

        # Register target_system now.
        self._handler.target_system = self._heartbeat_system

        # Wait until board has booted.
        while True:
            if self._flightmode not in [None, 'INITIALISING', 'MAV']:
                break
            time.sleep(0.1)

        # Initialize data stream.
        if rate is not None:
            self._master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                      rate, 1)

        self.add_message_listener('HEARTBEAT', self.send_capabilities_request)

        # Ensure initial parameter download has started.
        while True:
            # This fn actually rate limits itself to every 2s.
            # Just retry with persistence to get our first param stream.
            self._master.param_fetch_all()
            time.sleep(0.1)
            if self._params_count > -1:
                break

    def send_capabilties_request(self, vehicle, name, m):
        return self.send_capabilities_request(vehicle, name, m)

    def send_capabilities_request(self, vehicle, name, m):
        '''Request an AUTOPILOT_VERSION packet'''
        capability_msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(capability_msg)

    def play_tune(self, tune):
        '''Play a tune on the vehicle'''
        msg = self.message_factory.play_tune_encode(0, 0, tune)
        self.send_mavlink(msg)

    def wait_ready(self, *types, **kwargs):
        
        timeout = kwargs.get('timeout', 30)
        raise_exception = kwargs.get('raise_exception', True)

        # Vehicle defaults for wait_ready(True) or wait_ready()
        if list(types) == [True] or list(types) == []:
            types = self._default_ready_attrs

        if not all(isinstance(item, basestring) for item in types):
            raise ValueError('wait_ready expects one or more string arguments.')

        # Wait for these attributes to have been set.
        await_attributes = set(types)
        start = monotonic.monotonic()
        still_waiting_last_message_sent = start
        still_waiting_callback = kwargs.get('still_waiting_callback')
        still_waiting_message_interval = kwargs.get('still_waiting_interval', 1)

        while not await_attributes.issubset(self._ready_attrs):
            time.sleep(0.1)
            now = monotonic.monotonic()
            if now - start > timeout:
                if raise_exception:
                    raise TimeoutError('wait_ready experienced a timeout after %s seconds.' %
                                       timeout)
                else:
                    return False
            if (still_waiting_callback and
                    now - still_waiting_last_message_sent > still_waiting_message_interval):
                still_waiting_last_message_sent = now
                if still_waiting_callback:
                    still_waiting_callback(await_attributes - self._ready_attrs)

        return True

    def reboot(self):
        """Requests an autopilot reboot by sending a ``MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`` command."""

        reboot_msg = self.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
            0,  # confirmation
            1,  # param 1, autopilot (reboot)
            0,  # param 2, onboard computer (do nothing)
            0,  # param 3, camera (do nothing)
            0,  # param 4, mount (do nothing)
            0, 0, 0)  # param 5 ~ 7 not used

        self.send_mavlink(reboot_msg)

    


class Parameters(collections.MutableMapping, HasObservers):
    

    def __init__(self, vehicle):
        super(Parameters, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._vehicle = vehicle

    def __getitem__(self, name):
        name = name.upper()
        self.wait_ready()
        return self._vehicle._params_map[name]

    def __setitem__(self, name, value):
        name = name.upper()
        self.wait_ready()
        self.set(name, value)

    

    def __len__(self):
        return len(self._vehicle._params_map)

    def __iter__(self):
        return self._vehicle._params_map.__iter__()

    def get(self, name, wait_ready=True):
        name = name.upper()
        if wait_ready:
            self.wait_ready()
        return self._vehicle._params_map.get(name, None)

    def set(self, name, value, retries=3, wait_ready=False):
        if wait_ready:
            self.wait_ready()

        name = name.upper()
        # convert to single precision floating point number (the type used by low level mavlink messages)
        value = float(struct.unpack('f', struct.pack('f', value))[0])
        remaining = retries
        while True:
            self._vehicle._master.param_set_send(name, value)
            tstart = monotonic.monotonic()
            if remaining == 0:
                break
            remaining -= 1
            while monotonic.monotonic() - tstart < 1:
                if name in self._vehicle._params_map and self._vehicle._params_map[name] == value:
                    return True
                time.sleep(0.1)

        if retries > 0:
            self._logger.error("timeout setting parameter %s to %f" % (name, value))
        return False

    def wait_ready(self, **kwargs):

        self._vehicle.wait_ready('parameters', **kwargs)

    def add_attribute_listener(self, attr_name, *args, **kwargs):
        
        attr_name = attr_name.upper()
        return super(Parameters, self).add_attribute_listener(attr_name, *args, **kwargs)

    def remove_attribute_listener(self, attr_name, *args, **kwargs):
        
        attr_name = attr_name.upper()
        return super(Parameters, self).remove_attribute_listener(attr_name, *args, **kwargs)

    def notify_attribute_listeners(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super(Parameters, self).notify_attribute_listeners(attr_name, *args, **kwargs)

    def on_attribute(self, attr_name, *args, **kwargs):
        
        attr_name = attr_name.upper()
        return super(Parameters, self).on_attribute(attr_name, *args, **kwargs)


class Command(mavutil.mavlink.MAVLink_mission_item_message):
    
    pass


class CommandSequence(object):
   

    def __init__(self, vehicle):
        self._vehicle = vehicle

    def download(self):
       
        self.wait_ready()
        self._vehicle._ready_attrs.remove('commands')
        self._vehicle._wp_loaded = False
        self._vehicle._master.waypoint_request_list_send()
        # BIG FIXME - wait for full wpt download before allowing any of the accessors to work

    def wait_ready(self, **kwargs):
        
        return self._vehicle.wait_ready('commands', **kwargs)

    def clear(self):
       

        # Add home point again.
        self.wait_ready()
        home = None
        try:
            home = self._vehicle._wploader.wp(0)
        except:
            pass
        self._vehicle._wploader.clear()
        if home:
            self._vehicle._wploader.add(home, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True

    def add(self, cmd):
       
        self.wait_ready()
        self._vehicle._handler.fix_targets(cmd)
        self._vehicle._wploader.add(cmd, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True

    def upload(self, timeout=None):
        
        if self._vehicle._wpts_dirty:
            self._vehicle._master.waypoint_clear_all_send()
            start_time = time.time()
            if self._vehicle._wploader.count() > 0:
                self._vehicle._wp_uploaded = [False] * self._vehicle._wploader.count()
                self._vehicle._master.waypoint_count_send(self._vehicle._wploader.count())
                while False in self._vehicle._wp_uploaded:
                    if timeout and time.time() - start_time > timeout:
                        raise TimeoutError
                    time.sleep(0.1)
                self._vehicle._wp_uploaded = None
            self._vehicle._wpts_dirty = False

    @property
    def count(self):
        
        return max(self._vehicle._wploader.count() - 1, 0)

    @property
    def next(self):
        
        return self._vehicle._current_waypoint

    @next.setter
    def next(self, index):
        self._vehicle._master.waypoint_set_current_send(index)

    def __len__(self):

        return max(self._vehicle._wploader.count() - 1, 0)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[ii] for ii in range(*index.indices(len(self)))]
        elif isinstance(index, int):
            item = self._vehicle._wploader.wp(index + 1)
            if not item:
                raise IndexError('Index %s out of range.' % index)
            return item
        else:
            raise TypeError('Invalid argument type.')

    def __setitem__(self, index, value):
        self._vehicle._wploader.set(value, index + 1)
        self._vehicle._wpts_dirty = True


def default_still_waiting_callback(atts):
    logging.getLogger(__name__).debug("Still waiting for data from vehicle: %s" % ','.join(atts))


def connect(ip,
            _initialize=True,
            wait_ready=None,
            timeout=30,
            still_waiting_callback=default_still_waiting_callback,
            still_waiting_interval=1,
            status_printer=None,
            vehicle_class=None,
            rate=4,
            baud=115200,
            heartbeat_timeout=30,
            source_system=255,
            source_component=0,
            use_native=False):
    

    from dronekit.mavlink import MAVConnection

    if not vehicle_class:
        vehicle_class = Vehicle

    handler = MAVConnection(ip, baud=baud, source_system=source_system, source_component=source_component, use_native=use_native)
    vehicle = vehicle_class(handler)

    if status_printer:
        vehicle._autopilot_logger.addHandler(ErrprinterHandler(status_printer))

    if _initialize:
        vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)

    if wait_ready:
        if wait_ready is True:
            vehicle.wait_ready(still_waiting_interval=still_waiting_interval,
                               still_waiting_callback=still_waiting_callback,
                               timeout=timeout)
        else:
            vehicle.wait_ready(*wait_ready)

    return vehicle

