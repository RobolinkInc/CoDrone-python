from operator import eq
from queue import Queue
from threading import RLock, Thread, current_thread, Timer
from time import sleep
import colorama
from colorama import Fore, Back, Style
import serial
from serial.tools.list_ports import comports
import os.path
from CoDrone.receiver import *
from CoDrone.storage import *
from CoDrone.protocol import *
import matplotlib.pyplot as plt
import time


def convert_byte_array_to_string(data_array):
    if data_array is None:
        return ""

    string = ""

    if (isinstance(data_array, bytes)) or (isinstance(data_array, bytearray)) or (not isinstance(data_array, list)):
        for data in data_array:
            string += "{0:02X} ".format(data)

    return string


def _boundary_check(value,low=0,high=100):
    return low < value < high


class _Plot:
    def __init__(self):
        self._plot_setup = False
        self.height_flag = False
        self.height_axes = type(plt.Axes)
        self.pressure_flag = False
        self.pressure_axes = type(plt.Axes)
        self.temp_flag = False
        self.temp_axes = type(plt.Axes)
        self.angle_flag = False
        self.angle_roll_axes = type(plt.Axes)
        self.angle_pitch_axes = type(plt.Axes)
        self.angle_yaw_axes = type(plt.Axes)
        self.imu_gyro_flag = False
        self.imu_gyro_roll_axes = type(plt.Axes)
        self.imu_gyro_pitch_axes = type(plt.Axes)
        self.imu_gyro_yaw_axes = type(plt.Axes)
        self.imu_accel_flag = False
        self.imu_accel_x_axes = type(plt.Axes)
        self.imu_accel_y_axes = type(plt.Axes)
        self.imu_accel_z_axes = type(plt.Axes)
        self.opt_flag = False
        self.opt_axes = type(plt.Axes)
        self.motor_flag = False
        self.motor1_axes = type(plt.Axes)
        self.motor2_axes = type(plt.Axes)
        self.motor3_axes = type(plt.Axes)
        self.motor4_axes = type(plt.Axes)






class CoDrone:

    def __init__(self, flag_check_background=True, flag_show_error_message=False, flag_show_log_message=False,
                 flag_show_transfer_data=False, flag_show_receive_data=False):

        self._serialPort = None
        self._bufferQueue = Queue(4096)
        self._bufferHandler = bytearray()
        self._index = 0
        self._st = time.time()

        # Thread
        self._threadReceiving = None
        self._threadSendState = None
        self._lock = RLock()
        self._lockState = None
        self._lockReceiving = None
        self._flagThreadRun = False

        # Object
        self._receiver = Receiver()
        self._control = Control()

        # Flags
        self._flagCheckBackground = flag_check_background
        self._flagShowErrorMessage = flag_show_error_message
        self._flagShowLogMessage = flag_show_log_message
        self._flagShowTransferData = flag_show_transfer_data
        self._flagShowReceiveData = flag_show_receive_data

        # Handler
        self._eventHandler = EventHandler()
        self._storageHeader = StorageHeader()
        self._storage = Storage()
        self._storageCount = StorageCount()
        self._parser = Parser()

        self._devices = []  # when using auto connect, save search list
        self._flagDiscover = False  # when using auto connect, notice is discover
        self._flagConnected = False  # when using auto connect, notice connection with device
        self.Nearest = "0000"
        self.timeStartProgram = time.time()  # record program starting time

        # Data
        self._timer = Timer()
        self._data = Data(self._timer)
        self._set_all_event_handler()

        # Plot
        self._plot = _Plot()
        self._plot_time = None
        self._sensor_list = set()


        # Parameter
        self._lowBatteryPercent = 30    # when the program starts, battery alert percentage

        colorama.init()

    def __del__(self):
        self.close()

    # DATA PROCESSING THREAD -------- START

    def _receiving(self, lock, lock_state):
        """Data receiving Thread, Save received data to buffer.

        Args:
            lock: main thread lock
            lock_state: _sendRequestState lock
        """
        self._lockReceiving = RLock()
        while self._flagThreadRun:
            # lock other threads for reading
            # print('receive : ' + (self._st - time.time()).__str__())
            with lock and lock_state and self._lockReceiving:
                self._bufferQueue.put(self._serialPort.read())

            # auto-update when background check for receive data is on
            if self._flagCheckBackground:
                while self._check() != DataType.None_:
                    pass
                    # sleep(0.001)

    def _check(self):
        """Read 1-byte of data stored in the buffer and pass it to the receiver.

        Returns: A member value in the DataType class.
            If one data block received call _handler(), data parsing and return the datatype.
            Returns DataType.None_ if no data received.
        """
        while not self._bufferQueue.empty():
            data_array = self._bufferQueue.get_nowait()
            self._bufferQueue.task_done()

            if (data_array is not None) and (len(data_array) > 0):
                # print receive data
                self._print_receive_data(data_array)
                self._bufferHandler.extend(data_array)

        while len(self._bufferHandler) > 0:
            state_loading = self._receiver.call(self._bufferHandler.pop(0))

            # print error
            if state_loading == StateLoading.Failure:
                # print receive data
                self._print_receive_data_end()

                # print error
                self._print_error(self._receiver.message)

            # print log
            if state_loading == StateLoading.Loaded:
                # print receive data
                self._print_receive_data_end()

                # print log
                self._print_log(self._receiver.message)

            if self._receiver.state == StateLoading.Loaded:
                self._handler(self._receiver.header, self._receiver.data)
                return self._receiver.header.dataType

        return DataType.None_

    def _handler(self, header, data_array):
        """Save header internally. Data parsing and saved internal class.
        If event handler is registered, call function.

        Returns: A member value in the DataType class.
        """

        self._run_handler(header, data_array)

        # run callback event
        self._run_event_handler(header.dataType)

        # count number of request
        self._storageCount.d[header.dataType] += 1

        # process LinkEvent separately(event check like connect or disconnect)
        if (header.dataType == DataType.LinkEvent) and (self._storage.d[DataType.LinkEvent] is not None):
            self._event_link_event(self._storage.d[DataType.LinkEvent])

        # process LinkEventAddress separately(event check like connect or disconnect)
        if (header.dataType == DataType.LinkEventAddress) and (self._storage.d[DataType.LinkEventAddress] is not None):
            self._event_link_event_address(self._storage.d[DataType.LinkEventAddress])

        # process LinkDiscoveredDevice separately(add list of searched device)
        if (header.dataType == DataType.LinkDiscoveredDevice) and (
                self._storage.d[DataType.LinkDiscoveredDevice] is not None):
            self._event_link_discovered_device(self._storage.d[DataType.LinkDiscoveredDevice])

        # complete data process
        self._receiver.checked()

        return header.dataType

    def _run_handler(self, header, dataArray):
        """Store header and data into instance variables.
        """
        if self._parser.d[header.dataType] is not None:
            self._storageHeader.d[header.dataType] = header
            self._storage.d[header.dataType] = self._parser.d[header.dataType](dataArray)

    def _run_event_handler(self, dataType):
        """Call event handler with specified type of data
        """
        if (isinstance(dataType, DataType)) and (self._eventHandler.d[dataType] is not None) and (
                self._storage.d[dataType] is not None):
            return self._eventHandler.d[dataType](self._storage.d[dataType])
        else:
            return None

    def _set_all_event_handler(self):
        """Set all event handlers for SENSORS part functions.
        """
        self._eventHandler.d[DataType.Address] = self._data.eventUpdateAddress
        self._eventHandler.d[DataType.Attitude] = self._data.eventUpdateAttitude
        self._eventHandler.d[DataType.Battery] = self._data.eventUpdateBattery
        self._eventHandler.d[DataType.Pressure] = self._data.eventUpdatePressure
        self._eventHandler.d[DataType.Range] = self._data.eventUpdateRange
        self._eventHandler.d[DataType.State] = self._data.eventUpdateState
        self._eventHandler.d[DataType.Imu] = self._data.eventUpdateImu
        self._eventHandler.d[DataType.TrimFlight] = self._data.eventUpdateTrim
        self._eventHandler.d[DataType.ImageFlow] = self._data.eventUpdateImageFlow
        self._eventHandler.d[DataType.Ack] = self._data.eventUpdateAck
        self._eventHandler.d[DataType.Motor] = self._data.eventUpdateMotor

    def _send_request_state(self, lock):
        """Data request Thread, Send state data request every 2 sec.
        Args:
            lock: main thread lock
        """
        self._lockState = RLock()
        while self._flagThreadRun:
            if self._flagConnected:
                with lock and self._lockState:
                    self.send_request(DataType.State)
                    sleep(0.01)
                    print('receive : ' + (self._st - time.time()).__str__())
            sleep(2)

    def _grab_sensor_in_background(self, lock):
        self._lockState = RLock()
        while self._flagThreadRun:
            if self._flagConnected:
                with lock and self._lockState:
                    self.send_request(DataType.State)
                    # print(current_thread().name)
                    sleep(0.03)
                    if self._plot._plot_setup:
                        self._request_sensor(*self._sensor_list)
            sleep(0.5)

    def _request_sensor(self, *args):
        for sensor in args:
            if not isinstance(sensor, DataType) and isinstance(sensor, PlotType):
                self._print_error(">>> Parameter Type Error")  # print error message
                sleep(0.03)
                continue
            self._get_data_while(sensor, [0, 0])
            sleep(0.03)

    def set_plot_sensor(self, *args, auto=False):
        if not self._plot._plot_setup:
            self._plot._plot_setup = True
            print("plot init")
            self._data.star_time = time.process_time()

        for sensor in args:
            if not isinstance(sensor, PlotType):
                print("error")
                self._print_error(">>> Parameter Type Error")  # print error message
                sleep(0.03)
                continue
            self._set_plot_data(sensor)
            # print(sensor)
            self._sensor_list.add(self._plottype_to_datatype(sensor))
            # print(self._sensor_list)
        sleep(1)

    def _plottype_to_datatype(self, sensor):
        if sensor == PlotType.angle:
            return DataType.Attitude
        elif sensor == PlotType.accel or sensor == PlotType.gyro:
            return DataType.Imu
        elif sensor == PlotType.pressure or sensor == PlotType.temperature:
            return DataType.Pressure
        elif sensor == PlotType.height:
            return DataType.Range
        elif sensor == PlotType.image_flow:
            return DataType.ImageFlow
        elif sensor == PlotType.motor:
            return DataType.Motor

    def draw_plot_sensor(self, draw=False):
        if self._plot.height_flag:
            self._plot.height_axes.plot(self._data.range_time, self._data.range, 'ro-' )
            # print(self._data.range_time)
            # print(self._data.range)
            # self._plot._plot_setup = False
        if self._plot.pressure_flag:
            self._plot.pressure_axes.plot(self._data.pressure_time, self._data.pressure, 'ro-')
        if self._plot.temp_flag:
            self._plot.temp_axes.plot(self._data.pressure_time, self._data.temperature, 'ro-')
        if self._plot.imu_accel_flag:
            self._plot.imu_accel_x_axes.plot(self._data.imu_time, self._data.accel_x, 'ro-')
            self._plot.imu_accel_y_axes.plot(self._data.imu_time, self._data.accel_y, 'bo-')
            self._plot.imu_accel_z_axes.plot(self._data.imu_time, self._data.accel_z, 'go-')
        if self._plot.imu_gyro_flag:
            self._plot.imu_gyro_roll_axes.plot(self._data.imu_time, self._data.gyro_roll, 'ro-')
            self._plot.imu_gyro_pitch_axes.plot(self._data.imu_time, self._data.gyro_pitch, 'bo-')
            self._plot.imu_gyro_yaw_axes.plot(self._data.imu_time, self._data.gyro_yaw, 'go-')
        if self._plot.angle_flag:
            self._plot.angle_roll_axes.plot(self._data.attitude_time, self._data.attitude_roll, 'ro-')
            self._plot.angle_pitch_axes.plot(self._data.attitude_time, self._data.attitude_pitch, 'bo-')
            self._plot.angle_yaw_axes.plot(self._data.attitude_time, self._data.attitude_yaw, 'go-')
        if self._plot.opt_flag:
            self._plot.opt_axes.plot(self._data.imageFlow_x, self._data.imageFlow_y, 'b-')
        if self._plot.motor_flag:
            self._plot.motor1_axes.plot(self._data.m1, self._data.motor_time, '-')
            self._plot.motor2_axes.plot(self._data.m2, self._data.motor_time, '-')
            self._plot.motor3_axes.plot(self._data.m3, self._data.motor_time, '-')
            self._plot.motor4_axes.plot(self._data.m4, self._data.motor_time, '-')
        plt.show()

    def draw_plot_continue(self):
        if self._plot.height_flag:
            self._plot.height_axes.plot(self._data.range_time, self._data.range, 'ro-' )
            # print(self._data.range_time)
            # print(self._data.range)
            # self._plot._plot_setup = False
        if self._plot.pressure_flag:
            self._plot.pressure_axes.plot(self._data.pressure_time, self._data.pressure, 'ro-')
        if self._plot.temp_flag:
            self._plot.temp_axes.plot(self._data.pressure_time, self._data.temperature, 'ro-')
        if self._plot.imu_accel_flag:
            self._plot.imu_accel_x_axes.plot(self._data.imu_time, self._data.accel_x, 'ro-')
            self._plot.imu_accel_y_axes.plot(self._data.imu_time, self._data.accel_y, 'bo-')
            self._plot.imu_accel_z_axes.plot(self._data.imu_time, self._data.accel_z, 'go-')
        if self._plot.imu_gyro_flag:
            self._plot.imu_gyro_roll_axes.plot(self._data.imu_time, self._data.gyro_roll, 'ro-')
            self._plot.imu_gyro_pitch_axes.plot(self._data.imu_time, self._data.gyro_pitch, 'bo-')
            self._plot.imu_gyro_yaw_axes.plot(self._data.imu_time, self._data.gyro_yaw, 'go-')
        if self._plot.angle_flag:
            self._plot.angle_roll_axes.plot(self._data.attitude_time, self._data.attitude_roll, 'ro-')
            self._plot.angle_pitch_axes.plot(self._data.attitude_time, self._data.attitude_pitch, 'bo-')
            self._plot.angle_yaw_axes.plot(self._data.attitude_time, self._data.attitude_yaw, 'go-')
        if self._plot.opt_flag:
            self._plot.opt_axes.plot(self._data.imageFlow_x, self._data.imageFlow_y, 'b-')
        plt.draw()

    def _set_plot_data(self, plot_type):
        axes3 = None
        axes = []
        if plot_type == PlotType.accel or plot_type == PlotType.gyro or plot_type == PlotType.angle:
            figure, (axes, axes2, axes3) = plt.subplots(3, sharex=False, sharey=True)
        elif plot_type == PlotType.motor:
            figure, ((axes, axes2), (axes3, axes4)) = plt.subplots(2, 2)
        else:
            # figure, axes = plt.subplot()
            figure = plt.figure()
            axes = figure.add_subplot(111)

        # figure.suptitle(plot_type._name_ + " data plot", fontsize=15)

        axes.grid(linestyle='-')
        axes.set_xlabel("time(ms)")
        if plot_type == PlotType.pressure:
            axes.set_title("pressure data plot")
            axes.set_ylabel("pressure(C level height)")
            self._plot.pressure_flag = True
            self._plot.pressure_axes = axes
        elif plot_type == PlotType.image_flow:
            axes.set_title("image flow sensor position data plot")
            axes.set_xlabel("position(mm)")
            axes.set_ylabel("position(mm)")
            axes.set_ylim(-1000, 1000)
            axes.set_xlim(-1000, 1000)
            self._plot.opt_flag = True
            self._plot.opt_axes = axes
        elif plot_type == PlotType.height:
            axes.set_title("Height data plot")
            axes.set_ylabel("Height(mm)")
            # axes.set_ylim(0, 2500)
            self._plot.height_flag = True
            self._plot.height_axes = axes
        elif plot_type == PlotType.temperature:
            axes.set_title("Temperature data plot")
            axes.set_ylabel("temperature(℃)")
            self._plot.temp_flag = True
            self._plot.temp_axes = axes

        if axes3 is not None:
            axes2.grid(linestyle='-')
            axes3.grid(linestyle='-')
            axes3.set_xlabel("time(ms)")
            if plot_type == PlotType.gyro:
                axes.set_title("Gyro Roll data Plot")
                axes.set_ylabel("roll raw output")
                axes2.set_title("Gyro Pitch data Plot")
                axes2.set_ylabel("pitch raw output")
                axes3.set_title("Gyro Yaw data Plot")
                axes3.set_ylabel("yaw raw data")

                self._plot.imu_gyro_flag = True
                self._plot.imu_gyro_roll_axes = axes
                self._plot.imu_gyro_pitch_axes = axes2
                self._plot.imu_gyro_yaw_axes = axes3

            elif plot_type == PlotType.accel:
                axes.set_title("Accelometer X data Plot")
                axes.set_ylabel("x raw output")
                axes2.set_title("Accelometer Y data Plot")
                axes2.set_ylabel("y raw output")
                axes3.set_title("Accelometer Z data Plot")
                axes3.set_ylabel("z raw data")

                self._plot.imu_accel_flag = True
                self._plot.imu_accel_x_axes = axes
                self._plot.imu_accel_y_axes = axes2
                self._plot.imu_accel_z_axes = axes3

            elif plot_type == PlotType.angle:
                axes.set_title("Angle Roll data Plot")
                axes.set_ylabel("roll angle (degree)")
                axes2.set_title("Angle Pitch data Plot")
                axes2.set_ylabel("pitch angle (degree)")
                axes3.set_title("Angle Yaw data Plot")
                axes3.set_ylabel("yaw angle (degree)")

                axes.set_ylim(-180, 180)
                axes2.set_ylim(-180, 180)
                axes3.set_ylim(-180, 180)
                self._plot.angle_flag = True
                self._plot.angle_roll_axes = axes
                self._plot.angle_pitch_axes = axes2
                self._plot.angle_yaw_axes = axes3

            elif plot_type == PlotType.motor:
                axes.set_ylabel("motor1 speed")
                axes2.set_xlabel("time(ms)")
                axes2.set_ylabel("motor2 speed")
                axes3.set_ylabel("motor4 speed")
                axes4.set_ylabel("motor3 speed")
                axes4.grid(linestyle='-')

                axes.set_ylim(0, 4096)
                axes2.set_ylim(0, 4096)
                axes3.set_ylim(0, 4096)
                axes4.set_ylim(0, 4096)
                self._plot.motor_flag = True
                self._plot.motor1_axes = axes
                self._plot.motor2_axes = axes2
                self._plot.motor3_axes = axes4
                self._plot.motor4_axes = axes3

    def print_motor_output(self):

        print('--------------time : {:4.2}---------------------'.format(self._data.motor_time[-1]))
        print('Front Left ({:4})\tFront Right ({:4})'.format(self._data.m1[-1], self._data.m2[-1]))
        print("Back  Left ({:4})\tBack  Right ({:4})\n".format(self._data.m4[-1], self._data.m3[-1]))

    def lockState(func):
        """This function is a decorator for thread-locking.
        If you apply this decorator to the function, the data request thread doesn't work while the function works.

        Examples:
            @lockState
            def func:
                pass
        """
        def wrapper(self, *args, **kwargs):
            with self._lockState:
                return func(self, *args, **kwargs)
        return wrapper

    # DATA PROCESSING THREAD -------- END

    # PRIVATE -------- START

    @staticmethod
    def _make_transfer_data_array(header, data):
        """Make transfer byte data array
        """
        if (header is None) or (data is None):
            return None

        if (not isinstance(header, Header)) or (not isinstance(data, ISerializable)):
            return None

        crc16 = CRC16.calc(header.toArray(), 0)
        crc16 = CRC16.calc(data.toArray(), crc16)

        data_array = bytearray()
        data_array.extend((0x0A, 0x55))
        data_array.extend(header.toArray())
        data_array.extend(data.toArray())
        data_array.extend(pack('H', crc16))

        return data_array

    def _transfer(self, header, data):
        """Transfer data
        """
        if not self.isOpen():
            return

        dataArray = self._make_transfer_data_array(header, data)
        with self._lockReceiving and self._lock and self._lockState:
            self._serialPort.write(dataArray)

        # print _transfer data
        self._print_transfer_data(dataArray)
        return dataArray

    @lockState
    def _check_ack(self, header, data, timeOnce=0.03, timeAll=0.2, count=5):
        """This function checks the ack response after the data transfer.
        If not received, repeat the data transfer depending on parameters.

        Args:
            timeOnce: The time interval between the retransmissions of data. The number of seconds as type float.
            timeAll: The time until the function ends. The number of seconds as type float.
            count: The number of transfers

        Returns: True if the transfer works well, False otherwise.
        """
        self._data.ack.dataType = 0
        flag = 1

        self._transfer(header, data)
        start_time = time.time()
        while self._data.ack.dataType != header.dataType:
            interval = time.time() - start_time
            # Break the loop if request time is over timeAll sec, send the request maximum flagAll times
            if interval > timeOnce * flag and flag < count:
                self._transfer(header, data)
                flag += 1
            elif interval > timeAll:
                self._print_error(">> Failed to receive ack : {}".format(header.dataType))
                break
            sleep(0.01)
        return self._data.ack.dataType == header.dataType

    def _event_link_handler(self, eventLink):
        if eventLink == EventLink.Scanning:
            self._devices.clear()
            self._flagDiscover = True

        elif eventLink == EventLink.ScanStop:
            self._flagDiscover = False

        elif eventLink == EventLink.Connected:
            self._flagConnected = True

        elif eventLink == EventLink.Disconnected:
            self._flagConnected = False

        # print log
        self._print_log(eventLink)

    def _event_link_event(self, data):
        self._event_link_handler(data.eventLink)

    def _event_link_event_address(self, data):
        self._event_link_handler(data.eventLink)

    def _event_link_discovered_device(self, data):
        self._devices.append(data)

        # print log
        self._print_log(
            "LinkDiscoveredDevice / {0} / {1} / {2} / {3}".format(data.index, convert_byte_array_to_string(data.address)
                                                                  , data.name, data.rssi))

    def _print_log(self, message):
        if self._flagShowLogMessage and message is not None:
            print(Fore.GREEN + "[{0:10.03f}] {1}".format((time.time() - self.timeStartProgram),
                                                         message) + Style.RESET_ALL)

    def _print_error(self, message):
        if self._flagShowErrorMessage and message is not None:
            print(
                Fore.RED + "[{0:10.03f}] {1}".format((time.time() - self.timeStartProgram), message) + Style.RESET_ALL)

    def _print_transfer_data(self, dataArray):
        if self._flagShowTransferData and (dataArray is not None) and (len(dataArray) > 0):
            print(Back.YELLOW + Fore.BLACK + convert_byte_array_to_string(dataArray) + Style.RESET_ALL)

    def _print_receive_data(self, dataArray):
        if self._flagShowReceiveData and (dataArray is not None) and (len(dataArray) > 0):
            print(Back.CYAN + Fore.BLACK + convert_byte_array_to_string(dataArray) + Style.RESET_ALL, end='')

    def _print_receive_data_end(self):
        if self._flagShowReceiveData:
            print("")

    # PRIVATE -------- END

    # PUBLIC COMMON -------- START

    def isOpen(self):
        """Serial port connection status return.

        Returns: True if port is opened, false otherwise.
        """
        if self._serialPort is not None:
            return self._serialPort.isOpen()
        else:
            return False

    def isConnected(self):
        """BLE connection status return.

        Returns: True if BLE is connected, false otherwise.
        """
        if not self.isOpen():
            return False
        else:
            return self._flagConnected

    def open(self, port_name="None"):
        """Open serial port. If not specify a port name, connect to the last detected device.

        Args: Serial port name such as "COM14"

        Returns: True if port is opened, false otherwise.
        """
        is_open = False
        if eq(port_name, "None"):
            nodes = comports()
            size = len(nodes)
            if size > 0:
                port_name = nodes[size - 1].device
            else:
                is_open = False
        try:

            self._serialPort = serial.Serial(
                port=port_name,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0)

            if self.isOpen():
                self._flagThreadRun = True
                # self._threadSendState = Thread(target=self._send_request_state, args=(self._lock,), daemon=True).start()
                self._threadSendState = Thread(target=self._grab_sensor_in_background, args=(self._lock,),
                                               daemon=True).start()
                self._threadReceiving = Thread(target=self._receiving, args=(self._lock, self._lockState,),
                                               daemon=True).start()

                # print log
                print(">> Found Port : [{0}]".format(port_name))
                is_open = True

            else:
                # print error message
                print(">> Could not open the serial port")
                self._print_error(">> Could not open the serial port.")
                is_open = False

        except IOError:
            print(">> Could not open the serial port")
            print(">> Please check if CoDrone Bluetooth module is connnected")
            print(">> and verify you have the USB Driver")
            exit()

        return is_open

    def close(self):
        """Close serial port
        """

        # print log
        if self.isOpen():
            self._print_log("Closing serial port.")

        # close thread
        if self._flagThreadRun:
            self._flagThreadRun = False
            sleep(0.01)

        for i in range(5):
            self.send_link_disconnect()
            sleep(0.01)

        while self.isOpen():
            self._serialPort.close()
            sleep(0.01)

    def pair(self, device_name="None", port_name="None", flag_system_reset=False):
        """If the serial port is not open, open the serial port,
        Search for CoDrone and connect it to the device with the strongest signal.

        Args:
            device_name: If specify a deviceName, Connect only when the specified device is discovered.
            port_name: Serial port name.
            flag_system_reset: Use to reset and start the first CoDrone LINK after the serial communication connection.

        Returns: True if connected, false otherwise.
        """
        attempts = 2
        # case for serial port is None(connect to last connection)
        if not self.isOpen():
            self.close()
            self.open(port_name)

        # if not connect with serial port print error and return
        if not self.isOpen():
            # print error
            self._print_error(">> Could not connect to serial port.")
            return False

        # system reset
        if flag_system_reset:
            self.send_link_system_reset()

        # ModeLinkBroadcast.Passive mode change
        self.send_link_mode_broadcast(ModeLinkBroadcast.Passive)
        sleep(0.1)
        #scan attempts = 2 times
        for reconnection in range(attempts):
            # start searching device
            self._devices.clear()
            self._flagDiscover = True
            self.send_link_discover_start()

            # wait for 5sec
            for i in range(50):
                sleep(0.1)
                if not self._flagDiscover:
                    break
                sleep(0.01)

            length = len(self._devices)
            closest_device = None

            # near by drone
            if eq(device_name, "0000") or (eq(device_name, "None") and not os.path.exists('PairInfo')):
                # If not specify a name, connect to the nearest device
                if length > 0:
                    closest_device = self._devices[0]

                    # If more than two device is found, select the closest device
                    if len(self._devices) > 1:
                        for i in range(len(self._devices)):
                            if closest_device.rssi < self._devices[i].rssi:
                                closest_device = self._devices[i]

                    # connect the device
                    self._flagConnected = False
                    self.send_link_connect(closest_device.index)

                    # wait for 5 seconds to connect the device
                    for i in range(300):
                        if self._flagConnected:
                            break
                        sleep(0.01)
                    sleep(1.2)

                else:
                    print("scanning for CoDrone")
                    self._print_error(">> Could not find CoDrone.")

            # using CoDrone number
            else:
                # check the name of connected device
                target_device = None
                try:
                    if eq(device_name, "None"):
                        f = open('PairInfo', 'r')
                        device_name = f.readline()
                        f.close()
                except FileNotFoundError:
                    print("PairInfo file not exist Please try with number")

                if len(self._devices) > 0:
                    if len(device_name) == 4:
                        for i in range(len(self._devices)):
                            if (len(self._devices[i].name) > 12) and (device_name == self._devices[i].name[8:12]):
                                target_device = self._devices[i]
                                break

                        if target_device is not None:
                            closest_device = target_device

                            # if find the device, connect the device
                            self._flagConnected = False
                            self.send_link_connect(target_device.index)

                            # wait for 5 seconds to connect the device
                            for i in range(50):
                                sleep(0.1)
                                if self._flagConnected:
                                    break

                            # connect and wait another 1.2 seconds.
                            sleep(1.2)

                        else:
                            self._print_error(">> Could not find " + device_name + ".")

                    else:
                        self._print_error(">> Device name length error(" + device_name + ").")

                else:
                    self._print_error(">> Could not find CoDrone.")

            if self._flagConnected:
                print(">> Found CoDrone")
                battery = self.get_battery_percentage()
                print(">> Drone : [{}]"
                      "\n>> Battery : [{}]".format(closest_device.name[8:12], battery))

                if battery < self._lowBatteryPercent:
                    print(">> Low Battery!!")
                f = open('PairInfo', 'w')
                f.write(closest_device.name[8:12])
                f.close()
                return self._flagConnected
            else:
                self._print_error(">> Trying to connect : {}/5".format(reconnection + 1))
                if reconnection == attempts-1:
                    print(">> Failed to find CoDrone")
                    self._print_error(">> Fail to connect.")
                    print(">> please bring CoDrone closer or make sure it is on")
                    self._flagConnected = False
                    exit()
                    return self._flagConnected
        return self._flagConnected

    def connect(self, device_name="None", port_name="None", flag_system_reset=False):
        """If the serial port is not open, open the serial port,
        Search for CoDrone and connect it to the device with the strongest signal.

        Args:
            device_name: If specify a deviceName, Connect only when the specified device is discovered.
            port_name: Serial port name.
            flag_system_reset: Use to reset and start the first CODRONE LINK after the serial communication connection.

        Returns: True if connected, false otherwise.
        """

        # case for serial port is None(connect to last connection)
        if not self.isOpen():
            self.close()
            self.open(port_name)
            sleep(0.1)

        # if not connect with serial port print error and return
        if not self.isOpen():
            # print error
            self._print_error(">> Could not connect to serial port.")
            return False

        # system reset
        if flag_system_reset:
            self.send_link_system_reset()
            sleep(3)

        # ModeLinkBroadcast.Passive mode change
        self.send_link_mode_broadcast(ModeLinkBroadcast.Passive)
        sleep(0.1)

        for reconnection in range(5):
            # start searching device
            self._devices.clear()
            self._flagDiscover = True
            self.send_link_discover_start()

            # wait for 5sec
            for i in range(50):
                sleep(0.1)
                if not self._flagDiscover:
                    break

            sleep(2)

            length = len(self._devices)
            closest_device = None

            if eq(device_name, "None"):
                # If not specify a name, connect to the nearest device
                if length > 0:
                    closest_device = self._devices[0]

                    # If more than two device is found, select the closest device
                    if len(self._devices) > 1:
                        for i in range(len(self._devices)):
                            if closest_device.rssi < self._devices[i].rssi:
                                closest_device = self._devices[i]

                    # connect the device
                    self._flagConnected = False
                    self.send_link_connect(closest_device.index)

                    # wait for 5 seconds to connect the device
                    for i in range(50):
                        sleep(0.1)
                        if self._flagConnected:
                            break
                    sleep(1.2)

                else:
                    self._print_error(">> Could not find CODRONE.")

            else:
                # check the name of connected device
                target_device = None

                if len(self._devices) > 0:
                    if len(device_name) == 4:
                        for i in range(len(self._devices)):
                            if (len(self._devices[i].name) > 12) and (device_name == self._devices[i].name[8:12]):
                                target_device = self._devices[i]
                                break

                        if target_device is not None:
                            closest_device = target_device

                            # if find the device, connect the device
                            self._flagConnected = False
                            self.send_link_connect(target_device.index)

                            # wait for 5 seconds to connect the device
                            for i in range(50):
                                sleep(0.1)
                                if self._flagConnected:
                                    break

                            # connect and wait another 1.2 seconds.
                            sleep(1.2)

                        else:
                            self._print_error(">> Could not find " + device_name + ".")

                    else:
                        self._print_error(">> Device name length error(" + device_name + ").")

                else:
                    self._print_error(">> Could not find CoDrone.")

            if self._flagConnected:
                battery = self.get_battery_percentage()
                try:
                    print(">> Drone : [{}]\n>> Battery : [{}]".format(closest_device.name[8:12], battery))
                except:
                    print(">> Battery : [{}]".format(battery))

                if battery < self._lowBatteryPercent:
                    print(">> Low Battery!!")
                sleep(3)
                return self._flagConnected
            else:
                self._print_error(">> Trying to connect : {}/5".format(reconnection + 1))
                if reconnection == 4:
                    self._print_error(">> Fail to connect.")
        return self._flagConnected

    def disconnect(self):
        """Disconnect the drone.
        """
        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDisconnect
        data.option = 0

        return self._transfer(header, data)

    # PUBLIC COMMON -------- END

    # SENDING -------- Start

    def send_ping(self):
        header = Header()

        header.dataType = DataType.Ping
        header.length = Ping.getSize()

        data = Ping()

        data.systemTime = 0

        return self._transfer(header, data)

    @lockState
    def send_request(self, data_type):
        """This function sends data request with specified datatype.

        Args:
            data_type: a member value in the DataType enum class.

        Examples:
            >>> send_request(DataType.State)

        Returns: True if responds well, false otherwise.
        """
        if not isinstance(data_type, DataType):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        header = Header()

        header.dataType = DataType.Request
        header.length = Request.getSize()

        data = Request()

        data.dataType = data_type
        return self._transfer(header, data)

    @lockState
    def send_control(self, roll, pitch, yaw, throttle):
        """This function sends control request.

        Args:
            roll: the power of the roll, which is an int from -100 to 100
            pitch: the power of the pitch, which is an int from -100 to 100
            yaw: the power of the yaw, which is an int from -100 to 100
            throttle: the power of the throttle, which is an int from -100 to 100

        Returns: True if responds well, false otherwise.
        """
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        time_start = time.time()

        receiving_flag = self._storageCount.d[DataType.Attitude]

        while (time.time() - time_start) < 0.2:
            self._transfer(header, control)
            sleep(0.02)
            if self._storageCount.d[DataType.Attitude] > receiving_flag:
                break
        if self._storageCount.d[DataType.Attitude] == receiving_flag:
            self._print_error(">> Failed to send control.")

        return self._storageCount.d[DataType.Attitude] == receiving_flag

    @lockState
    def send_control_duration(self, roll, pitch, yaw, throttle, duration):
        """This function sends control request for the duration

        Args:
            roll: the power of the roll, which is an int from -100 to 100
            pitch: the power of the pitch, which is an int from -100 to 100
            yaw: the power of the yaw, which is an int from -100 to 100
            throttle: the power of the throttle, which is an int from -100 to 100

        Returns: True if responds well, false otherwise.
        """

        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        self._transfer(header, control)

        time_start = time.time()
        while (time.time() - time_start) < duration:
            self._transfer(header, control)
            sleep(0.02)

        self.hover(1)

    # SENDING -------- End

    # FLIGHT VARIABLES -------- START
    """
    Setter:
        Args:
            power: An int from -100 to 100 that sets the variable.

    Getter:
        Returns: The power of the variable(int)        
    """

    def set_roll(self, power):
        self._control.roll = power

    def set_pitch(self, power):
        self._control.pitch = power

    def set_yaw(self, power):
        self._control.yaw = power

    def set_throttle(self, power):
        self._control.throttle = power

    def get_roll(self):
        return self._control.roll

    def get_pitch(self):
        return self._control.pitch

    def get_yaw(self):
        return self._control.yaw

    def get_throttle(self):
        return self._control.throttle

    def trim(self, roll, pitch, yaw, throttle):
        """This is a setter function that allows you to set the trim of the drone if it's drifting.

        Args:
            roll: An int from -100(left) to 100(right) that sets the roll trim.
            pitch: An int from -100(backward) to 100(forward) that sets the pitch trim.
            yaw: An int from -100(counterclockwise) to 100(clockwise) that sets the yaw trim.
            throttle: An int from -100(downward) to 100(upward) that sets the throttle trim.
        """
        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()
        data.setAll(roll, pitch, yaw, throttle)

        if not self._check_ack(header, data):
            self._print_error(">> Failed to trim")

    def reset_trim(self, power):
        """This is a setter function that allows you to set the throttle variable.

        Args:
            power: An int from -100 to 100 that sets the throttle variable.
            The number represents the direction and power of the output for that flight motion variable.
            Negative throttle descends, positive throttle ascends.
        """
        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()
        data.setAll(0, 0, 0, power)

        if not self._check_ack(header, data):
            self._print_error(">> Failed to reset trim")

    # FLIGHT VARIABLES -------- END

    # FLIGHT COMMANDS (START/STOP) -------- START

    def takeoff(self):
        """This function makes the drone take off and begin hovering.
        The drone will always hover for 3 seconds in order to stabilize before it executes the next command.
        If it receives no command for 8 seconds, it will automatically land.
        """
        self._data.takeoffFuncFlag = 1  # Event States

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.TakeOff.value

        if not self._check_ack(header, data):
            self._print_error(">> Failed to takeoff")
        sleep(3)

    def land(self):
        """This function makes the drone stop all commands, hovers, and makes a soft landing where it is.
        The function will also zero-out all of the flight motion variables to 0.
        """
        self._control.setAll(0, 0, 0, 0)  # set the flight motion variables to 0.

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.Landing.value

        if not self._check_ack(header, data):
            self._print_error(">> Failed to land")
        sleep(3)

    def hover(self, duration=0):
        """This function makes the drone hover for a given amount of time.

        Args:
            duration: The number of seconds to hover as type float. If 0, the duration is infinity.
        """
        timeStart = time.time()
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(0, 0, 0, 0)

        if duration != 0:
            while (time.time() - timeStart) < duration:
                self._transfer(header, control)
                sleep(0.1)
        else:
            if not self._check_ack(header, control):
                self._print_error(">> Failed to hover")

    def emergency_stop(self):
        """This function immediately stops all commands and stops all motors, so the drone will stop flying immediately.
        The function will also zero-out all of the flight motion variables to 0.
        """
        self._data.stopFuncFlag = 1  # Event states
        self._control.setAll(0, 0, 0, 0)  # set the flight motion variables to 0

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Stop
        data.option = 0

        if not self._check_ack(header, data):
            self._print_error(">> Failed to emergency stop")

    # FLIGHT COMMANDS (START/STOP) -------- END

    # FLIGHT COMMANDS (MOVEMENT) -------- START

    def calibrate(self):
        """This function sends control request.

        Args:

        Returns:
        """
        print("start")
        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Calibrate
        data.option = 0
        print("ready to send")
        return self._transfer(header, data)

    def move(self, *args):
        """Once flying, the drone goes in the direction of the set flight motion variables.
        If the drone is not flying, nothing will happen.
        If you provide no parameters or only duration, it will execute it based on the current flight motion variables set.
        Args:
            duration: the duration of the flight motion in seconds. If 0, the duration is infinity.
            roll: the power of the roll, which is an int from -100 to 100
            pitch: the power of the pitch, which is an int from -100 to 100
            yaw: the power of the yaw, which is an int from -100 to 100
            throttle: the power of the throttle, which is an int from -100 to 100

        Examples:
            >>> move()  #goes infinity with setter setting value
            >>> move(3)    #goes for 3 seconds setter setting value
            >>> move(0, 0, 0, 50) #goes upward at 50% power
            >>> move(3, 0, 0, 0, 50)   #goes upward for 3 seconds at 50% power
        """
        if len(args) == 0:  # move()
            self.send_control(*self._control.getAll())
            sleep(1)
        elif len(args) == 1:  # move(duration)
            self.send_control_duration(*self._control.getAll(), args[0])
        elif len(args) == 4:  # move(roll, pitch, yaw, throttle)
            self.send_control(*args)
        elif len(args) == 5:  # move(duration, roll, pitch, yaw, throttle)
            self.send_control_duration(args[1], args[2], args[3], args[4], args[0])

    def go(self, direction, duration=0, power=50):
        """A simpler Junior level function that represents positive flight with a direction, but with more natural language.
        It simply flies forward for the given duration and power.

        Args:
            direction: member values in the Direction enum class which can be one of the following: FORWARD, BACKWARD, LEFT, RIGHT, UP, and DOWN.
            duration: the duration of the flight motion in seconds.
                If 0, it will turn right indefinitely. Defaults to infinite if not defined.
            power: the power at which the drone flies. Takes a value from 0 to 100. Defaults to 50 if not defined.

        Examples:
            >>> go(Direction.FORWARD)   # goes forward infinitely at 50% power
            >>> go(Direction.UP, 3)    # goes up for 3 seconds at 50% power
            >>> go(Direction.BACKWARD, 3, 30)   # goes backward for 3 seconds at 30% power
        """
        # power or -power
        pitch = ((direction == Direction.FORWARD) - (direction == Direction.BACKWARD)) * power
        roll = ((direction == Direction.RIGHT) - (direction == Direction.LEFT)) * power
        throttle = ((direction == Direction.UP) - (direction == Direction.DOWN)) * power

        self.send_control_duration(roll, pitch, 0, throttle, duration)

    def turn(self, direction, duration=0, power=50):
        """A simpler Junior level function that represents yaw, but with more natural language.
        It simply turns in the given direction, duration and power.

        Args:
            direction: member values in the Direction enum class which can be one of the following: LEFT, RIGHT
            duration: the duration of the turn in seconds.
                If 0 or not defined, it will turn right indefinitely.
            power: the power at which the drone turns right. Takes a value from 0 to 100. Defaults to 50 if not defined.

        Examples:
            >>> turn(Direction.LEFT)    # yaws left infinitely at 50 power
            >>> turn(Direction.RIGHT, 3)    # yaws right for 3 seconds at 50 power
            >>> turn(Direction.RIGHT, 5, 100)   # yaws right for 5 seconds at 100 power
        """
        yaw = ((direction == Direction.RIGHT) - (direction == Direction.LEFT)) * power
        if duration is None:
            self.send_control(0, 0, yaw, 0)
        else:
            self.send_control_duration(0, 0, yaw, 0, duration)

    @lockState
    def turn_degree(self, direction, degree):
        """An Senior level function that yaws by a given degree in a given direction.
        This function takes an input degree in an input direction, and turns until it reaches the given degree.

        Args:
            direction: member values in the Direction enum class which can be one of the following: LEFT, RIGHT
            degree: member values in the Degree enum class which can be one of the following:
                ANGLE_30, ANGLE_45, ANGLE_60, ANGLE_90, ANGLE_120, ANGLE_135, ANGLE_150, ANGLE_180

        Examples:
            >>> turn_degree(Direction.LEFT, Degree.ANGLE_30)    # turn left 30 degrees
        """
        if not isinstance(direction, Direction) or not isinstance(degree, Degree):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        power = 20
        bias = 3

        yaw_past = self.get_gyro_angles().YAW
        direction = ((direction == Direction.RIGHT) - (direction == Direction.LEFT))  # right = 1 / left = -1
        degree_goal = direction * (degree.value - bias) + yaw_past

        start_time = time.time()
        while (time.time() - start_time) < degree.value / 3:
            yaw = self._data.attitude.YAW  # Receive attitude data every time you send a flight command
            if abs(yaw_past - yaw) > 180:  # When the sign changes
                degree_goal -= direction * 360
            yaw_past = yaw
            if direction > 0 and degree_goal > yaw:  # Clockwise
                self.send_control(0, 0, power, 0)
            elif direction < 0 and degree_goal < yaw:  # Counterclockwise
                self.send_control(0, 0, -power, 0)
            else:
                break
            sleep(0.05)

        self.send_control(0, 0, 0, 0)
        sleep(1)

    @lockState
    def rotate180(self):
        """This function makes the drone rotate 180 degrees.
        """
        power = 20
        bias = 3

        yaw_past = self.get_gyro_angles().YAW
        degree_goal = 180 - bias + yaw_past

        start_time = time.time()
        while (time.time() - start_time) < 60:
            yaw = self._data.attitude.YAW  # Receive attitude data every time you send a flight command
            if abs(yaw_past - yaw) > 180:  # When the sign changes
                degree_goal -= 360
            yaw_past = yaw
            if degree_goal > yaw:  # Clockwise
                self.send_control(0, 0, power, 0)
            else:
                break
            sleep(0.05)

    @lockState
    def go_to_height(self, height):
        """This is a setter function will make the drone fly to the given height above the object directly below its IR sensor (usually the ground).
        It’s effective between 20 and 1500 millimeters.
        It uses the IR sensor to continuously check for its height.

        height: An int from 20 to 2000 in millimeters.
        """
        power = 30
        interval = 20  # height - 10 ~ height + 10

        start_time = time.time()
        while time.time() - start_time < 100:
            state = self.get_height()
            differ = height - state
            if differ > interval:  # Up
                self.send_control(0, 0, 0, power)
                sleep(0.1)
            elif differ < -interval:  # Down
                self.send_control(0, 0, 0, -power)
                sleep(0.1)
            else:
                break

        self.send_control(0, 0, 0, 0)
        sleep(1)

    # FLIGHT COMMANDS (MOVEMENT) -------- END

    # SENSORS -------- START

    @lockState
    def _get_data_while(self, data_type, timer=None):
        """This function checks if a request arrived or not and requests again maximum 3 times, 0.15sec

        Args:
            data_type: member values in the DataType class
            timer: member values in the Timer class
        """
        time_start = time.time()

        if timer is not None:
            if timer[0] > (time_start - timer[1]):
                return False

        header = Header()
        header.dataType = DataType.Request
        header.length = Request.getSize()

        data = Request()
        data.dataType = data_type

        # Break the loop if request time is over 0.15sec, send the request maximum 3 times
        receiving_flag = self._storageCount.d[data_type]
        resend_flag = 1
        self._transfer(header, data)
        while self._storageCount.d[data_type] == receiving_flag:
            interval = time.time() - time_start
            if interval > 0.03 * resend_flag and resend_flag < 3:
                self._transfer(header, data)
                resend_flag += 1
            elif interval > 0.15:
                break
            sleep(0.01)
        return self._storageCount.d[data_type] > receiving_flag

    def get_height(self):
        """This is a getter function gets the current height of the drone from the object directly below its IR sensor.

        Returns:  The current height above the object directly below the drone’s IR height sensor.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Range, self._timer.range)
        return self._data.range[-1]

    def get_pressure(self):
        """This is a getter function gets the data from the barometer sensor.

        Returns: The barometer’s air pressure in milibars at (0.13 resolution).
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Pressure, self._timer.pressure)
        return self._data.pressure[-1]

    def get_drone_temp(self):
        """This is a getter function gets the data from the drone’s temperature sensor.
        Importantly, it reads the drone’s temperature, not the air around it.

        Returns: The temperature in celsius as an integer.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Pressure, self._timer.pressure)
        return self._data.temperature[-1]

    def get_angular_speed(self):
        """This function gets the data from the gyrometer sensor for the roll, pitch, and yaw angular speed.

        Returns: The Angle class. Angle has ROLL, PITCH, YAW.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Imu, self._timer.imu)
        return self._data.gyro

    def get_gyro_angles(self):
        """This function gets the data from the gyrometer sensor to determine the roll, pitch, and yaw as angles.

        Returns: The Angle class. Angle has ROLL, PITCH, YAW.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Attitude, self._timer.attitude)
        return self._data.attitude

    def get_accelerometer(self):
        """This function gets the accelerometer sensor data, which returns x, y, and z values in m/s2.

        Returns: The Axis class. Axis has X,Y,Z
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Imu, self._timer.imu)
        return self._data.accel

    def get_opt_flow_position(self):
        """This function gets the x and y coordinates from the optical flow sensor.

        Returns: The Position class. Position has X,Y
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.ImageFlow, self._timer.imageFlow)
        return self._data.imageFlow

    def get_state(self):
        """This function gets the state of the drone, as in whether it’s: ready, take off, flight, flip, stop, landing, reverse, accident, error

        Returns: string of member values in the ModeFlight class.
            READY, TAKE_OFF, FLIGHT, FLIP, STOP, LANDING, REVERSE, ACCIDENT, ERROR

        Examples:
            >>>print(getState())
            Ready
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.State, self._timer.state)
        return self._data.state.name

    def get_battery_percentage(self):
        """This function gets the battery percentage of the drone.

        Returns: The battery’s percentage as an integer from 0 - 100.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Battery, self._timer.battery)
        return self._data.batteryPercent

    def get_battery_voltage(self):
        """This function gets the voltage of the battery.

        Returns: The voltage of the battery as an a float
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.Battery, self._timer.battery)
        return self._data.batteryVoltage

    def get_trim(self):
        """This function gets the current trim values of the drone.

        Returns: The Flight class. Flight has ROLL, PITCH, YAW, THROTTLE
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._get_data_while(DataType.TrimFlight, self._timer.trim)
        return self._data.trim

    def get_motor_input(self):
        self._get_data_while(DataType.Motor, self._timer.trim)
        print(self._data.motor.__str__())
        return self._data.motor

    # SENSORS -------- END

    # STATUS CHECKERS -------- START
    def is_upside_down(self):
        """This function checks the current drone status if it's reversed or not.

        Returns:
             boolean: True if upside down, False otherwise.
        """
        return self._data.reversed == SensorOrientation.Normal

    def is_flying(self):
        """This function checks the current drone status if it's flying or not.

        Returns:
             boolean: True if flying, False otherwise.
        """
        return self._data.state == ModeFlight.FLIGHT

    def is_ready_to_fly(self):
        """This function checks the current drone status if it's ready or not.

        Returns:
             boolean: True if ready to fly, False otherwise.
        """
        return self._data.state == ModeFlight.READY

    # STATUS CHECKERS -------- END

    # EVENT STATES -------- START

    def on_upside_down(self, func):
        """This function executes the function if drone is reversed.

        Args: A function.

        Example:
             def func():
                pass
             onUpsideDown(func)
        """
        self._data.upsideDown = func

    def on_takeoff(self, func):
        """This function executes the function if drone takeoff.

        Args: A function.

        Example:
             def func():
                pass
             onTakeoff(func)
        """
        self._data.takeoff = func

    def on_flying(self, func):
        """This function executes the function if drone is on flying.

        Args: A function.

        Example:
             def func():
                pass
             onFlying(func)
        """
        self._data.flying = func

    def on_ready(self, func):
        """This function executes the function if drone is on ready.

        Args: A function.

        Example:
             def func():
                pass
             onReady(func)
        """
        self._data.ready = func

    def on_emergency_stop(self, func):
        """This function executes the function if drone is on emergency stop.

        Args: A function.

        Example:
             def func():
                pass
             onEmergencyStop(func)
        """
        self._data.emergencyStop = func

    def on_low_battery(self, func):
        """This function executes the function if drone is on low battery.

        Args: A function.

        Example:
             def func():
                pass
             onLowBattery(func)
        """
        self._data.lowBattery = func

    # EVENT STATES -------- END

    # LEDS -------- START

    @lockState
    def send_led_process(self, dataType, *args):

        length = len(args)

        # check datatype parameter
        if ((not isinstance(dataType, DataType)) or
                (not isinstance(args[0], LightModeDrone))):
            return None

        # mode data object with right dataType class
        if dataType is DataType.LightMode:
            data = LightMode()
        elif dataType is DataType.LightModeColor or dataType is DataType.LightModeDefaultColor:
            data = LightModeColor()
        elif dataType is DataType.LightMode2:
            data = LightMode2()
        elif dataType is DataType.LightModeColor2 or dataType is DataType.LightModeDefaultColor2:
            data = LightModeColor2()
        else:
            self._print_error(">>>Data type not support yet")
            return None

        # generate Header object
        header = Header()
        header.dataType = dataType
        header.length = data.getSize()

        # assign data to object
        # length 3 is for LightMode 5 is for LightModeColor or LightModeDefaultColor
        # 6 for LightMode2 and 10 for LightModeColor2 or LightModeDefaultColor2
        if length == 3:
            data.mode = args[0]
            data.color = args[1]
            data.interval = args[2]

        elif length == 5:
            data.mode = args[0]
            data.color.r = args[1]
            data.color.g = args[2]
            data.color.b = args[3]
            data.interval = args[4]

        elif length == 6:
            data.lightMode1.mode = args[0]
            data.lightMode1.color = args[1]
            data.lightMode1.interval = args[2]
            data.lightMode2.mode = args[3]
            data.lightMode2.color = args[4]
            data.lightMode2.interval = args[5]

        elif length == 10:
            data.lightModeColor1.mode = args[0]
            data.lightModeColor1.color.r = args[1]
            data.lightModeColor1.color.g = args[2]
            data.lightModeColor1.color.b = args[3]
            data.lightModeColor1.interval = args[4]
            data.lightModeColor2.mode = args[5]
            data.lightModeColor2.color.r = args[6]
            data.lightModeColor2.color.g = args[7]
            data.lightModeColor2.color.b = args[8]
            data.lightModeColor2.interval = args[9]

        return self._check_ack(header, data, 0.06, 0.3, 5)

    def set_eye_led(self, r=-1, g=-1, b=-1, mode=-1, interval=100):
        """
        This function sets the LED color of the eyes, the light pattern, and the interval of the
        light pattern. You can set the color based on input red, green, and blue values or using
        predefined colors.

        Syntax:
            set_eye_led(color, mode)
            set_eye_led(color, mode, interval)
            set_eye_led(red, green, blue, mode)
            set_eye_led(red, green, blue, mode, interval)

        Arguments:
            r: int value from 0 to 255, or type Color
            g: int value from 0 to 255, or type Mode
            b: int value from 0 to 255
            mode: an enum, which can be selected from the following predefined list: SOLID, STROBE, BLINK, DOUBLE_BLINK, DIMMING, PULSE, REVERSE_PULSE, OFF
            interval: the interval of the light pattern, except for in the case of SOLID. For SOLID mode, this refers to the light's brightness.

        Returns:
            None
        """
        if isinstance(r, Color):

            if not isinstance(g, Mode):
                self._print_error(">>> Parameter Type Error")  # print error message
                return
            elif b not in range(-1, 256):
                self._print_error(">>> boundary exception")  # print error message
                return

            mode = g
            interval = b if (b > 0) else 100

            self.send_led_process(DataType.LightMode, LightModeDrone(mode.value), r, interval)

        elif isinstance(r, int):

            if not isinstance(mode, Mode):
                self._print_error(">>> Parameter Type Error")  # print error message
                return
            elif r not in range(0, 256) or g not in range(0, 256) or b not in range(0, 256) \
                    or interval not in range(0, 256):
                self._print_error(">>> boundary exception")  # print error message
                return

            self.send_led_process(DataType.LightModeColor, LightModeDrone(mode.value), r, g, b, interval)
        else:
            self._print_error(">>> Parameter Error")  # print error message

    def set_arm_led(self, r=-1, g=-1, b=-1, mode=-1, interval=100):
        """
        This function sets the LED color of the arms, the light pattern, and the interval of the
        light pattern. You can set the color based on input red, green, and blue values or using
        predefined colors.

        Syntax:
            set_arm_led(color, mode)
            set_arm_led(color, mode, interval)
            set_arm_led(red, green, blue, mode)
            set_arm_led(red, green, blue, mode, interval)

        Arguments:
            r: int value from 0 to 255, or type Color
            g: int value from 0 to 255, or type Mode
            b: int value from 0 to 255
            mode: an enum, which can be selected from the following predefined list: SOLID, STROBE, BLINK, DOUBLE_BLINK, DIMMING, PULSE, REVERSE_PULSE, OFF
            interval: the interval of the light pattern, except for in the case of SOLID. For SOLID mode, this refers to the light's brightness.

        Returns:
            None
        """
        if isinstance(r, Color):

            if not isinstance(g, Mode):
                self._print_error(">>> Parameter Type Error")  # print error message
                return
            elif b not in range(-1, 256):
                self._print_error(">>> boundary exception")  # print error message
                return
            mode = g
            interval = b if (b > 0) else 100
            self.send_led_process(DataType.LightMode, LightModeDrone(mode.value + 0x30), r, interval)

        elif isinstance(r, int):

            if not isinstance(mode, Mode):
                self._print_error(">>> Parameter Type Error")  # print error message
                return
            elif r not in range(0, 256) or g not in range(0, 256) or b not in range(0, 256) \
                    or interval not in range(0, 256):
                self._print_error(">>> boundary exception")  # print error message
                return

            self.send_led_process(DataType.LightModeColor, LightModeDrone(mode.value + 0x30), r, g, b, interval)

        else:
            self._print_error(">>> Parameter Error")  # print error message

    def set_all_led(self, r=-1, g=-1, b=-1, mode=-1, interval=100):
        """
        This function sets the LED color of both the arms and eyes, the light pattern, and the interval of the
        light pattern. You can set the color based on input red, green, and blue values or using predefined
        colors.

        Syntax:
            set_all_led(color, mode)
            set_all_led(color, mode, interval)
            set_all_led(red, green, blue, mode)
            set_all_led(red, green, blue, mode, interval)

        Arguments:
            r: int value from 0 to 255, or type Color
            g: int value from 0 to 255, or type Mode
            b: int value from 0 to 255
            mode: an enum, which can be selected from the following predefined list: SOLID, STROBE, BLINK, DOUBLE_BLINK, DIMMING, PULSE, REVERSE_PULSE, OFF
            interval: the interval of the light pattern, except for in the case of SOLID. For SOLID mode, this refers to the light's brightness.

        Returns:
            None
         """

        if isinstance(r, Color):
            if not isinstance(g, Mode):
                self._print_error(">>> Parameter Type Error")  # print error message
                return
            elif b not in range(-1, 256):
                self._print_error(">>> boundary exception")  # print error message
                return

            mode = g
            interval = b if (b > 0) else 100
            self.send_led_process(DataType.LightMode2, LightModeDrone(mode.value), r, interval,
                                  LightModeDrone(mode.value + 0x30), r, interval)

        elif isinstance(r, int):
            if not isinstance(mode, Mode):
                self._print_error(">>> Parameter Type Error")  # print error message
                return
            elif r not in range(0, 256) or g not in range(0, 256) or b not in range(0, 256) \
                    or interval not in range(0, 256):
                self._print_error(">>> boundary exception")  # print error message
                return

            self.send_led_process(DataType.LightModeColor2, LightModeDrone(mode.value), r, g, b, interval,
                                  LightModeDrone(mode.value + 0x30), r, g, b, interval)
        else:
            self._print_error(">>> Parameter Error")  # print error message

    def set_eye_default_led(self, r=-1, g=-1, b=-1, mode=-1, interval=100):
        """
        This function sets the default LED color of the eyes as well as the mode, so it will remain that color
        even after powering off and back on. The colors set are using RGB values.

        Syntax:
            set_eye_default_led(red, green, blue, mode)
            set_eye_default_led(red, green, blue, mode, interval)

        Arguments:
            r: int value from 0 to 255, or type Color
            g: int value from 0 to 255, or type Mode
            b: int value from 0 to 255
            mode: an enum, which can be selected from the following predefined list: SOLID, STROBE, BLINK, DOUBLE_BLINK, DIMMING, PULSE, REVERSE_PULSE, OFF
            interval: the interval of the light pattern, except for in the case of SOLID. For SOLID mode, this refers to the light's brightness.

        Returns:
            None
        """
        if not isinstance(mode, Mode):
            self._print_error(">>> Parameter Type Error")  # print error message
            return
        elif r not in range(0, 256) or g not in range(0, 256) or b not in range(0, 256) \
                or interval not in range(0, 256):
            self._print_error(">>> boundary exception")  # print error message
            return

        self.send_led_process(DataType.LightModeDefaultColor, LightModeDrone(mode.value), r, g, b, interval)

    def set_arm_default_led(self, r=-1, g=-1, b=-1, mode=-1, interval=100):
        """
        This function sets the default LED color of the arms as well as the mode, so it will remain that color
        even after powering off and back on. The colors are set using RGB values.

        Syntax:
            set_arm_default_led(red, green, blue, mode)
            set_arm_default_led(red, green, blue, mode, interval)

        Arguments:
            r: int value from 0 to 255
            g: int value from 0 to 255
            b: int value from 0 to 255
            mode: an enum, which can be selected from the following predefined list: SOLID, STROBE, BLINK, DOUBLE_BLINK, DIMMING, PULSE, REVERSE_PULSE, OFF
            interval: the interval of the light pattern, except for in the case of SOLID. For SOLID mode, this refers to the light's brightness.

        Returns:
            None
        """

        if not isinstance(mode, Mode):
            self._print_error(">>> Parameter Type Error")  # print error message
            return
        elif r not in range(0, 256) or g not in range(0, 256) or b not in range(0, 256) \
                or interval not in range(0, 256):
            self._print_error(">>> boundary exception")  # print error message
            return

        self.send_led_process(DataType.LightModeDefaultColor, LightModeDrone(mode.value + 0x30), r, g, b, interval)

    def set_all_default_led(self, r=-1, g=-1, b=-1, mode=-1, interval=100):
        """
        This function sets the default LED color of the eyes and arms as well as the mode, so it will remain that
        color and light pattern even after powering off and back on. The colors are set using RGB values.

        Syntax:
            set_all_default_led(red, green, blue, mode)
            set_all_default_led(red, green, blue, mode, interval)

        Arguments:
            r: int value from 0 to 255
            g: int value from 0 to 255
            b: int value from 0 to 255
            mode: an enum, which can be selected from the following predefined list: SOLID, STROBE, BLINK, DOUBLE_BLINK, DIMMING, PULSE, REVERSE_PULSE, OFF
            interval: the interval of the light pattern, except for in the case of SOLID. For SOLID mode, this refers to the light's brightness.

        Returns:
            None
        """

        if not isinstance(mode, Mode):
            self._print_error(">>> Parameter Type Error")  # print error message
            return
        elif r not in range(0, 256) or g not in range(0, 256) or b not in range(0, 256) \
                or interval not in range(0, 256):
            self._print_error(">>> boundary exception")  # print error message
            return
        self.send_led_process(DataType.LightModeDefaultColor2, LightModeDrone(mode.value), r, g, b, interval,
                              LightModeDrone(mode.value + 0x30), r, g, b, interval)

    def reset_default_led(self):
        """This function sets the LED color of the eyes and arms back to red, which is the original default color.
        """
        try:
            self.send_led_process(DataType.LightModeDefaultColor2, LightModeDrone.ArmHold, 255, 0, 0, 100,
                                  LightModeDrone.EyeHold, 255, 0, 0, 100)
        except:
            self._print_error("you put wrong value")
        finally:
            self.close()

    def arm_color(self, r=-1, g=-1, b=-1, brightness=-1):
        try:
            if isinstance(r, Color):
                brightness = g
                self.send_led_process(DataType.LightMode, LightModeDrone(Mode.SOLID.value + 0x30),r,brightness)
            else:
                self.send_led_process(DataType.LightModeColor, LightModeDrone(Mode.SOLID.value + 0x30),
                                      r, g, b, brightness)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def eye_color(self, r=-1, g=-1, b=-1, brightness=-1):
        try:
            if isinstance(r, Color):
                brightness = g
                self.send_led_process(DataType.LightMode, LightModeDrone(Mode.SOLID.value), r, brightness)
            else:
                self.send_led_process(DataType.LightModeColor, LightModeDrone(Mode.SOLID.value), r, g, b, brightness)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def all_colors(self, r=-1, g=-1, b=-1, brightness=-1):
        try:
            if isinstance(r, Color):
                brightness = g
                self.send_led_process(DataType.LightMode2, LightModeDrone(Mode.SOLID.value), r, brightness,
                                      LightModeDrone(Mode.SOLID.value + 0x30), r, brightness)
            else:
                self.send_led_process(DataType.LightModeColor2, LightModeDrone(Mode.SOLID.value), r, g, b, brightness,
                                      LightModeDrone(Mode.SOLID.value + 0x30), r, g, b, brightness)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def arm_default_color(self, r=-1, g=-1, b=-1, brightness=-1):
        try:
            self.send_led_process(DataType.LightModeDefaultColor, LightModeDrone(Mode.SOLID.value + 0x30),
                                      r, g, b, brightness)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def eye_default_color(self, r=-1, g=-1, b=-1, brightness=-1):
        try:
            self.send_led_process(DataType.LightModeDefaultColor, LightModeDrone(Mode.SOLID.value),
                              r, g, b, brightness)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def arm_pattern(self, r=-1, g=-1, b=-1, pattern=-1, interval=-1):
        try:
            if isinstance(r, Color):
                pattern = g
                interval = b

                self.send_led_process(DataType.LightMode, LightModeDrone(pattern.value + 0x30), r, interval)
            else:
                self.send_led_process(DataType.LightModeColor, LightModeDrone(pattern.value + 0x30), r, g, b, interval)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def eye_pattern(self, r=-1, g=-1, b=-1, pattern=-1, interval=-1):
        try:
            if isinstance(r, Color):
                pattern = g
                interval = b
                self.send_led_process(DataType.LightMode, LightModeDrone(pattern.value), r, interval)
            else:
                self.send_led_process(DataType.LightModeColor, LightModeDrone(pattern.value), r, g, b, interval)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def arm_default_pattern(self, r=-1, g=-1, b=-1, pattern=-1, interval=-1):
        try:
            self.send_led_process(DataType.LightModeDefaultColor, LightModeDrone(pattern.value + 0x30),
                                  r, g, b, interval)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def eye_default_pattern(self, r=-1, g=-1, b=-1, pattern=-1, interval=-1):
        try:
            self.send_led_process(DataType.LightModeDefaultColor, LightModeDrone(pattern.value), r, g, b, interval)
        except:
            self._print_error(">>> you put wrong parameter")  # print error message

    def arm_strobe(self):
        self.send_led_process(DataType.LightMode, LightModeDrone(Mode.STROBE.value + 0x30), Color.White, 0)

    def eye_strobe(self):
        self.send_led_process(DataType.LightMode, LightModeDrone(Mode.STROBE.value), Color.White, 0)

    def arm_off(self):
        self.send_led_process(DataType.LightMode, LightModeDrone(Mode.OFF.value + 0x30), Color.White, 0)

    def eye_off(self):
        self.send_led_process(DataType.LightMode, LightModeDrone(Mode.OFF.value), Color.White, 0)

    # LEDS --------- END

    # FLIGHT SEQUENCES -------- START

    def fly_sequence(self, sequence):
        """This function makes the drone fly in a given pattern, then land.

        Args:
            Member values in the Sequence class. Sequence class has SQUARE, CIRCLE, SPIRAL, TRIANGLE, HOP, SWAY, ZIG_ZAG
        """
        if sequence == Sequence.SQUARE:
            self.fly_square()
        elif sequence == Sequence.CIRCLE:
            self.fly_circle()
        elif sequence == Sequence.SPIRAL:
            self.fly_spiral()
        elif sequence == Sequence.TRIANGLE:
            self.fly_triangle()
        elif sequence == Sequence.HOP:
            self.fly_hop()
        elif sequence == Sequence.SWAY:
            self.fly_sway()
        elif sequence == Sequence.ZIGZAG:
            self.fly_zigzag()
        else:
            return None

    def fly_roulette(self):
        """This function makes yaw for a random number of seconds between 5 and 10, then pitch forward in that direction.
        """

        self.turn(Direction.RIGHT, 5 + (self.timeStartProgram % 5), 30)
        self.go(Direction.FORWARD, 1)

        self.hover(1)

    def turtle_turn(self):
        """If the drone is in the upside down state.
        This function makes the drone turn right side up by spinning the right two propellers
        """
        self.go(Direction.UP, 1, 100)

    def fly_square(self):

        self.go(Direction.RIGHT, 2, 30)
        self.go(Direction.FORWARD, 2, 30)
        self.go(Direction.LEFT, 2, 30)
        self.go(Direction.BACKWARD, 2, 30)

        self.hover(1)

    def fly_circle(self):

        self.move(0, 40, 0, 0, 0)
        sleep(0.2)
        self.move(0, 40, 0, -60, 0)
        sleep(2.5)
        self.move(0, 40, 0, -50, 0)
        sleep(1.0)
        self.move(0, 30, 0, 0, 0)
        sleep(0.1)

        self.hover(1)

    def fly_spiral(self):

        for i in range(5):
            self.send_control(10 + 2 * i, 0, -50, 0)
            sleep(1)

        self.hover(1)

    def fly_triangle(self):

        self.turn_degree(Direction.RIGHT, Degree.ANGLE_30)
        self.go(Direction.FORWARD, 2, 30)
        self.turn_degree(Direction.LEFT, Degree.ANGLE_120)
        self.go(Direction.FORWARD, 2, 30)
        self.turn_degree(Direction.LEFT, Degree.ANGLE_120)
        self.go(Direction.FORWARD, 2, 30)

        self.hover(1)

    def fly_hop(self):

        self.send_control_duration(0, 30, 0, 50, 1)
        self.send_control_duration(0, 30, 0, -50, 1)

        self.hover(1)

    def fly_sway(self):

        for i in range(2):
            self.go(Direction.LEFT, 1, 50)
            self.go(Direction.RIGHT, 1, 50)

        self.hover(1)

    def fly_zigzag(self):

        for i in range(2):
            self.move(1, 50, 50, 0, 0)
            self.move(1, -50, 50, 0, 0)

        self.hover(1)

    # FLIGHT SEQUENCES -------- END

    # Link -------- Start

    def send_link_mode_broadcast(self, mode_link_broadcast):
        if not isinstance(mode_link_broadcast, ModeLinkBroadcast):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkModeBroadcast
        data.option = mode_link_broadcast.value

        return self._transfer(header, data)

    def send_link_system_reset(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkSystemReset
        data.option = 0

        return self._transfer(header, data)

    def send_link_discover_start(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDiscoverStart
        data.option = 0

        return self._transfer(header, data)

    def send_link_discover_stop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDiscoverStop
        data.option = 0

        return self._transfer(header, data)

    def send_link_connect(self, index):

        if not isinstance(index, int):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkConnect
        data.option = index

        return self._transfer(header, data)

    def send_link_disconnect(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDisconnect
        data.option = 0

        return self._transfer(header, data)

    def send_link_rssi_polling_start(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkRssiPollingStart
        data.option = 0

        return self._transfer(header, data)

    def send_link_rssi_polling_stop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkRssiPollingStop
        data.option = 0

        return self._transfer(header, data)

    # LINK -------- END

    def set_event_handler(self, dataType, eventHandler):
        if not isinstance(dataType, DataType):
            return

        self._eventHandler.d[dataType] = eventHandler

    def get_header(self, dataType):
        if not isinstance(dataType, DataType):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        return self._storageHeader.d[dataType]

    def get_data(self, dataType):
        if not isinstance(dataType, DataType):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        return self._storage.d[dataType]

    def get_count(self, dataType):

        if not isinstance(dataType, DataType):
            self._print_error(">>> Parameter Type Error")  # print error message
            return None

        return self._storageCount.d[dataType]

    # LEGACY CODE -------- START

    def send_take_off(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.TakeOff.value

        return self._transfer(header, data)

    def send_landing(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.Landing.value

        return self._transfer(header, data)

    def send_stop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Stop
        data.option = 0

        return self._transfer(header, data)

    def send_control_while(self, roll, pitch, yaw, throttle, time_ms):

        if (not isinstance(roll, int)) or (not isinstance(pitch, int)) or (not isinstance(yaw, int)) or \
                (not isinstance(throttle, int)):
            return None

        time_sec = time_ms / 1000
        time_start = time.time()

        while (time.time() - time_start) < time_sec:
            self.send_control(roll, pitch, yaw, throttle)
            sleep(0.02)

        return self.send_control(roll, pitch, yaw, throttle)

    def send_control_drive(self, wheel, speed):

        if (not isinstance(wheel, int)) or (not isinstance(speed, int)):
            return None

        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        data = Control()

        data.roll = speed
        data.pitch = 0
        data.yaw = 0
        data.throttle = wheel

        return self._transfer(header, data)

    def send_control_drive_while(self, wheel, speed, time_ms):

        if (not isinstance(wheel, int)) or (not isinstance(speed, int)):
            return None

        time_sec = time_ms / 1000
        time_start = time.time()

        while (time.time() - time_start) < time_sec:
            self.send_control_drive(wheel, speed)
            sleep(0.02)

        return self.send_control_drive(wheel, speed)

    # Control End

    # Setup Start

    def send_command(self, command_type, option=0):

        if (not isinstance(command_type, CommandType)) or (not isinstance(option, int)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = command_type
        data.option = option

        return self._transfer(header, data)

    def send_mode_vehicle(self, mode_vehicle):

        if not isinstance(mode_vehicle, ModeVehicle):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ModeVehicle
        data.option = mode_vehicle.value

        return self._transfer(header, data)

    def send_headless(self, headless):

        if not isinstance(headless, Headless):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Headless
        data.option = headless.value

        return self._transfer(header, data)

    def send_trim(self, trim):

        if not isinstance(trim, Trim):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Trim
        data.option = trim.value

        return self._transfer(header, data)

    def send_trim_flight(self, roll, pitch, yaw, throttle):

        if ((not isinstance(roll, int)) or (not isinstance(pitch, int)) or (not isinstance(yaw, int)) or (
                not isinstance(throttle, int))):
            return None

        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()

        data.roll = roll
        data.pitch = pitch
        data.yaw = yaw
        data.throttle = throttle

        return self._transfer(header, data)

    def send_trim_drive(self, wheel):

        if not isinstance(wheel, int):
            return None

        header = Header()

        header.dataType = DataType.TrimDrive
        header.length = TrimDrive.getSize()

        data = TrimDrive()

        data.wheel = wheel

        return self._transfer(header, data)

    def send_flight_event(self, flight_event):

        if (not isinstance(flight_event, FlightEvent)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = flight_event.value

        return self._transfer(header, data)

    def send_drive_event(self, drive_event):

        if (not isinstance(drive_event, DriveEvent)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.DriveEvent
        data.option = drive_event.value

        return self._transfer(header, data)

    def send_clear_trim(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ClearTrim
        data.option = 0

        return self._transfer(header, data)

    def send_clear_gyro_bias(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ClearGyroBias
        data.option = 0

        return self._transfer(header, data)

    def send_update_lookup_target(self, device_type):

        if not isinstance(device_type, DeviceType):
            return None

        header = Header()

        header.dataType = DataType.UpdateLookupTarget
        header.length = UpdateLookupTarget.getSize()

        data = UpdateLookupTarget()

        data.deviceType = device_type

        return self._transfer(header, data)

    # Setup End

    # Device Start

    def send_motor(self, motor0, motor1, motor2, motor3):

        if ((not isinstance(motor0, int)) or
                (not isinstance(motor1, int)) or
                (not isinstance(motor2, int)) or
                (not isinstance(motor3, int))):
            return None

        header = Header()

        header.dataType = DataType.Motor
        header.length = Motor.getSize()

        data = Motor()

        data.motor[0].forward = motor0
        data.motor[0].reverse = 0

        data.motor[1].forward = motor1
        data.motor[1].reverse = 0

        data.motor[2].forward = motor2
        data.motor[2].reverse = 0

        data.motor[3].forward = motor3
        data.motor[3].reverse = 0

        return self._transfer(header, data)

    def send_ir_message(self, value):

        if not isinstance(value, int):
            return None

        header = Header()

        header.dataType = DataType.IrMessage
        header.length = IrMessage.getSize()

        data = IrMessage()

        data.irData = value

        return self._transfer(header, data)

    # Device End

    # Light Start

    def send_light_mode(self, light_mode, colors, interval):

        if ((not isinstance(light_mode, LightModeDrone)) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Color))):
            return None

        header = Header()

        header.dataType = DataType.LightMode
        header.length = LightMode.getSize()

        data = LightMode()

        data.mode = light_mode
        data.color = colors
        data.interval = interval

        return self._transfer(header, data)

    def send_light_mode_command(self, light_mode, colors, interval, commandType, option):

        if ((not isinstance(light_mode, LightModeDrone)) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Color)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeCommand
        header.length = LightModeCommand.getSize()

        data = LightModeCommand()

        data.lightMode.mode = light_mode
        data.lightMode.colors = colors
        data.lightMode.interval = interval

        data.command.commandType = commandType
        data.command.option = option

        return self._transfer(header, data)

    def send_light_mode_command_ir(self, light_mode, interval, colors, commandType, option, irData):

        if ((not isinstance(light_mode, LightModeDrone)) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Color)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(irData, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeCommandIr
        header.length = LightModeCommandIr.getSize()

        data = LightModeCommandIr()

        data.lightMode.mode = light_mode
        data.lightMode.colors = colors
        data.lightMode.interval = interval

        data.command.commandType = commandType
        data.command.option = option

        data.irData = irData

        return self._transfer(header, data)

    def send_light_mode_color(self, light_mode, r, g, b, interval):

        if ((not isinstance(light_mode, LightModeDrone)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int)) or
                (not isinstance(interval, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = light_mode
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval

        return self._transfer(header, data)

    def send_light_event(self, light_event, colors, interval, repeat):

        if ((not isinstance(light_event, LightModeDrone)) or
                (not isinstance(colors, Color)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEvent
        header.length = LightEvent.getSize()

        data = LightEvent()

        data.event = light_event
        data.color = colors
        data.interval = interval
        data.repeat = repeat

        return self._transfer(header, data)

    def send_light_event_command(self, light_event, colors, interval, repeat, commandType, option):

        if ((not isinstance(light_event, LightModeDrone)) or
                (not isinstance(colors, Color)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEventCommand
        header.length = LightEventCommand.getSize()

        data = LightEventCommand()

        data.lightEvent.event = light_event
        data.lightEvent.colors = colors
        data.lightEvent.interval = interval
        data.lightEvent.repeat = repeat

        data.command.commandType = commandType
        data.command.option = option

        return self._transfer(header, data)

    def send_light_event_command_ir(self, light_event, colors, interval, repeat, commandType, option, irData):

        if ((not isinstance(light_event, LightModeDrone)) or
                (not isinstance(colors, Color)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(irData, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEventCommandIr
        header.length = LightEventCommandIr.getSize()

        data = LightEventCommandIr()

        data.lightEvent.event = light_event
        data.lightEvent.colors = colors
        data.lightEvent.interval = interval
        data.lightEvent.repeat = repeat

        data.command.commandType = commandType
        data.command.option = option

        data.irData = irData

        return self._transfer(header, data)

    def send_light_event_color(self, light_event, r, g, b, interval, repeat):

        if (((not isinstance(light_event, LightModeDrone))) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEventColor
        header.length = LightEventColor.getSize()

        data = LightEventColor()

        data.event = light_event.mode
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval
        data.repeat = repeat

        return self._transfer(header, data)

    def send_light_mode_default_color(self, light_mode, r, g, b, interval):

        if ((not isinstance(light_mode, LightModeDrone)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int)) or
                (not isinstance(interval, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()

        data.mode = light_mode
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval

        return self._transfer(header, data)

    def send_light_mode2(self, light_mode1, color1, interval1, light_mode2, color2, interval2):

        if ((not isinstance(light_mode1, LightModeDrone)) or
                (not isinstance(interval1, int)) or
                (not isinstance(color1, Color)) or
                (not isinstance(light_mode2, LightModeDrone)) or
                (not isinstance(interval2, int)) or
                (not isinstance(color2, Color))):
            return None

        header = Header()

        header.dataType = DataType.LightMode2
        header.length = LightMode2.getSize()

        data = LightMode2()

        data.lightMode1.mode = light_mode1
        data.lightMode1.color = color1
        data.lightMode1.interval = interval1
        data.lightMode2.mode = light_mode2
        data.lightMode2.color = color2
        data.lightMode2.interval = interval2

        return self._transfer(header, data)

    def send_light_mode_color2(self, light_mode1, r1, g1, b1, interval1, light_mode2, r2, g2, b2, interval2):

        if ((not isinstance(light_mode1, LightModeDrone)) or
                (not isinstance(r1, int)) or
                (not isinstance(g1, int)) or
                (not isinstance(b1, int)) or
                (not isinstance(interval1, int)) or
                (not isinstance(light_mode2, LightModeDrone)) or
                (not isinstance(r2, int)) or
                (not isinstance(g2, int)) or
                (not isinstance(b2, int)) or
                (not isinstance(interval2, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor2
        header.length = LightModeColor2.getSize()

        data = LightModeColor2()

        data.lightModeColor1.mode = light_mode1
        data.lightModeColor1.color.r = r1
        data.lightModeColor1.color.g = g1
        data.lightModeColor1.color.b = b1
        data.lightModeColor1.interval = interval1
        data.lightModeColor2.mode = light_mode2
        data.lightModeColor2.color.r = r2
        data.lightModeColor2.color.g = g2
        data.lightModeColor2.color.b = b2
        data.lightModeColor2.interval = interval2

        return self._transfer(header, data)

    # Light End
    # LEGACY CODE -------- END
