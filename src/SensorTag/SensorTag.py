import bgapi.module
import time
import serial.tools.list_ports

from enum import Enum

class SensorTagServices(Enum):
    Device              = "1800"
    DeviceInformation   = "180A"
    Battery             = "180F"
    Button              = "FFE0"
    IRTemp              = "F000AA00-0451-4000-B000-000000000000"
    Humidity            = "F000AA20-0451-4000-B000-000000000000"
    Barometer           = "F000AA40-0451-4000-B000-000000000000"
    IO                  = "F000AA64-0451-4000-B000-000000000000"
    Luxometer           = "F000AA70-0451-4000-B000-000000000000"
    Movement            = "F000AA80-0451-4000-B000-000000000000"
    Register            = "F000AC00-0451-4000-B000-000000000000"
    Display             = "F000AD00-0451-4000-B000-000000000000"
    ConnectionControl   = "F000CCC0-0451-4000-B000-000000000000"

class DeviceCharacteristics(Enum):
    Name                            = "2A00"
    Appearance                      = "2A01"
    PreferredConnectionParameters   = "2A04"

class DevicePreferredConnectionParameters():
    MinConnectionIntervalLSB = 0
    MinConnectionIntervalMSB = 1
    MaxConnectionIntervalLSB = 2
    MaxConnectionIntervalMSB = 3
    SlaveLatencyLSB = 4
    SlaveLatencyMSB = 5
    SupervisionTimeoutLSB = 6
    SupervisionTimeoutMSB = 7

class DeviceInformationCharacteristics(Enum):
    SystemID            = "2A23"
    ModelNumber         = "2A24"
    SerialNumber        = "2A25"
    FirmwareRevision    = "2A26"
    HardwareRevision    = "2A27"
    SoftwareRevision    = "2A28"
    ManufacturerName    = "2A29"
    PnPID               = "2A50"

class DeviceInformationPnPID():
    CompanyLSB = 1
    CompanyMSB = 2
    ProductIDLSB = 3
    ProductIDMSB = 4
    ProductVersionLSB = 5
    ProductVersionMSB = 6

class BatteryCharacteristics(Enum):
    BatteryLevel    = "2A19"

class ButtonCharacteristics(Enum):
    Data            = "FFE1"

class ButtonData():
    UserButtonMask      = 0b001
    UserButtonShift     = 0
    PowerButtonMask     = 0b010
    PowerButtonShift    = 1
    ReedRelayMask       = 0b100
    ReedRelayShift      = 2

class IRTempCharacteristics(Enum):
    Data            = "F000AA01-0451-4000-B000-000000000000"
    Configuration   = "F000AA02-0451-4000-B000-000000000000"
    Period          = "F000AA03-0451-4000-B000-000000000000"

class IRTempData():
    Length = 4
    ObjectLSB   = 0
    ObjectMSB   = 1
    AmbienceLSB = 2
    AmbienceMSB = 3

class IRTempConfiguration(Enum):
    Disable = 0
    Enable  = 1

class IRTempPeriod():
    Min = 0x1E
    Max = 0xFF

class HumidityCharacteristics(Enum):
    Data            = "F000AA21-0451-4000-B000-000000000000"
    Configuration   = "F000AA22-0451-4000-B000-000000000000"
    Period          = "F000AA23-0451-4000-B000-000000000000"

class HumidityData():
    Length = 4
    TempLSB = 0
    TempMSB = 1
    HumLSB  = 2
    HumMSB  = 3

class HumidityConfiguration(Enum):
    Disable = 0
    Enable  = 1

class HumidityPeriod():
    Min = 0x0A
    Max = 0xFF

class BarometerCharacteristics(Enum):
    Data            = "F000AA41-0451-4000-B000-000000000000"
    Configuration   = "F000AA42-0451-4000-B000-000000000000"
    Period          = "F000AA44-0451-4000-B000-000000000000"

class BarometerData():
    Length = 6
    TempB0  = 0
    TempB1  = 1
    TempB2  = 2
    PressB0 = 3
    PressB1 = 4
    PressB2 = 5

class BarometerConfiguration(Enum):
    Disable = 0
    Enable = 1

class BarometerPeriod():
    Min = 0x0A
    Max = 0xFF

class IOCharacteristics(Enum):
    Data            = "F000AA65-0451-4000-B000-000000000000"
    Configuration   = "F000AA66-0451-4000-B000-000000000000"

class IODataRemote():
    Length = 1
    RedMask     = 0b00000001
    GreenMask   = 0b00000010
    BuzzerMask  = 0b00000100

class IODataTest():
    Length = 1
    IRTempMask          = 0b00000001
    HumidityMask        = 0b00000010
    LuxometerMask       = 0b00000100
    BarometerMask       = 0b00001000
    MovementMask        = 0b00010000
    MagnetometerMask    = 0b00100000
    ExternalFlashMask   = 0b01000000
    DevPackMask         = 0b10000000

class IOMode(Enum):
    Local   = 0
    Remote  = 1
    Test    = 2

class LuxometerCharacteristics(Enum):
    Data            = "F000AA71-0451-4000-B000-000000000000"
    Configuration   = "F000AA72-0451-4000-B000-000000000000"
    Period          = "F000AA73-0451-4000-B000-000000000000"

class LuxometerData():
    Length = 2
    LightLSB = 0
    LightMSB = 1

class LuxometerConfiguration(Enum):
    Disable = 0
    Enable  = 1

class LuxometerPeriod():
    Min = 0x0A
    Max = 0xFF

class MovementCharacteristics(Enum):
    Data            = "F000AA81-0451-4000-B000-000000000000"
    Configuration   = "F000AA82-0451-4000-B000-000000000000"
    Period          = "F000AA83-0451-4000-B000-000000000000"

class MovementData():
    Length = 18
    GyroXLSB    = 0
    GyroXMSB    = 1
    GyroYLSB    = 2
    GyroYMSB    = 3
    GyroZLSB    = 4
    GyroZMSB    = 5
    AccXLSB     = 6
    AccXMSB     = 7
    AccYLSB     = 8
    AccYMSB     = 9
    AccZLSB     = 10
    AccZMSB     = 11
    MagXLSB     = 12
    MagXMSB     = 13
    MagYLSB     = 14
    MagYMSB     = 15
    MagZLSB     = 16
    MagZMSB     = 17

class MovementConfiguration():
    Length = 2
    LSB = 0
    MSB = 1
    GyroZMask           = 0b0000000000000001
    GyroYMask           = 0b0000000000000010
    GyroXMask           = 0b0000000000000100
    AccZMask            = 0b0000000000001000
    AccYMask            = 0b0000000000010000
    AccXMask            = 0b0000000000100000
    MagMask             = 0b0000000001000000
    WakeOnMotionMask    = 0b0000000010000000
    AccRangeMask        = 0b0000001100000000
    AccRangeShift       = 8

class AccelerometerRange(Enum):
    G2  = 0
    G4  = 1
    G8  = 2
    G16 = 3

class MovementPeriod():
    Min = 0x0A
    Max = 0xFF

class RegisterCharacteristics(Enum):
    Data        = "F000AC01-0451-4000-B000-000000000000"
    Address     = "F000AC02-0451-4000-B000-000000000000"
    DeviceID    = "F000AC03-0451-4000-B000-000000000000"

class RegisterData():
    MinLength = 1
    MaxLength = 4

class RegisterAddress():
    Length          = 5
    Size            = 0
    Address         = 1
    AddressLength   = 4

class RegisterDeviceID():
    Length      = 2
    InterfaceID = 0
    Address     = 1

class RegisterInterfaceIDs():
    I2C0    = 0
    I2C1    = 1
    SPI     = 2
    MCU     = 5

class RegisterAddresses(Enum):
    Humidity        = RegisterInterfaceIDs.I2C0, 0x43
    IRTemp          = RegisterInterfaceIDs.I2C0, 0x44
    Luxometer       = RegisterInterfaceIDs.I2C0, 0x45
    Barometer       = RegisterInterfaceIDs.I2C0, 0x77
    Magnetometer    = RegisterInterfaceIDs.I2C1, 0x0B
    Movement        = RegisterInterfaceIDs.I2C1, 0x68
    SPI             = RegisterInterfaceIDs.SPI
    MCU             = RegisterInterfaceIDs.MCU

class DisplayCharacteristics(Enum):
    Data    = "F000AD01-0451-4000-B000-000000000000"
    Control = "F000AD02-0451-4000-B000-000000000000"

class DisplayData():
    MinLength = 1
    MaxLength = 16

class DisplayControl():
    Command = 0
    Data0   = 1
    Data1   = 2

class DisplayCommands():
    DisplayOff          = 1
    DisplayOffData      = 0
    DisplayOn           = 2
    DisplayOnData       = 0
    ClearAll            = 3
    ClearAllData        = 0
    ClearLine           = 4
    ClearLineData       = 1
    ClearLineRow        = 1
    InvertColor         = 5
    InvertColorData     = 0
    SetPosition         = 6
    SetPositionData     = 2
    SetPositionRow      = 0
    SetPositionColumn   = 1

class DisplayTextAlign():
    No      = 0
    Left    = 1
    Center  = 2
    Right   = 3

class DisplayLimit():
    Row = 12
    Column = 16

class ConnectionControlCharacteristics(Enum):
    ConnectionParameters        = "F000CCC1-0451-4000-B000-000000000000"
    RequestConnectionParameters = "F000CCC2-0451-4000-B000-000000000000"
    RequestDisconnect           = "F000CCC3-0451-4000-B000-000000000000"

class ConnectionControlData():
    Length = 6
    ConnectionIntervalLSB   = 0
    ConnectionIntervalMSB   = 1
    SlaveLatencyLSB         = 2
    SlaveLatencyMSB         = 3
    SupervisionTimeoutLSB   = 4
    SupervisionTimeoutMSB   = 5

class ConnectionControlRequest():
    Length = 8
    MaxConnectionIntervalLSB    = 0
    MaxConnectionIntervalMSB    = 1
    MinConnectionIntervalLSB    = 2
    MinConnectionIntervalMSB    = 3
    SlaveLatencyLSB             = 4
    SlaveLatencyMSB             = 5
    SupervisionTimeoutLSB       = 6
    SupervisionTimeoutMSB       = 7

class ConnectionControlDisconnect():
    Length = 1
    Disconnect = 0

class SensorTag:
    def __init__(self, port: str, target: list, baud: int = 250000) -> None:
        self.port = port
        self.target = target
        self.targetReversed = target.copy()
        self.targetReversed.reverse()
        self.targetBytes: bytes = bytes(self.targetReversed)
        self.baud = baud
        self.client = bgapi.module.BlueGigaClient(self.port, self.baud)
        self.connection = None
        self.connected = False
        self.device = None
        self.devices = None
        self.services: dict = {}
        self.maximumAttempts = 10
        self.client.reset_ble_state()
        self._UUIDH2B16 = lambda uuid : int(uuid.replace('-', ''), 16).to_bytes(16, byteorder="little")
        self._UUIDH2B2 = lambda uuid : int(uuid.replace('-', ''), 16).to_bytes(2, byteorder="little")
    
    @staticmethod
    def ports():
        ports = serial.tools.list_ports.comports()
        for port in ports:
            yield port.name, port.device, port.description

    def scan(self, timeout: int = 3):
        self.devices = self.client.scan_all(timeout)
        for device in self.devices:
            if self.targetBytes == device.sender:
                self.device = device
    
    def connect(self):
        if self.devices == None or self.device == None:
            raise Exception('Scan BLE devices first')
        else:
            self.connection = self.client.connect(self.device, 5, 6, 6, 100, 0)
            self.connected = True
            self.connection.read_by_group_type(bgapi.module.GATTService.PRIMARY_SERVICE_UUID, timeout = 10)
            services = self.connection.get_services()
            for service in services:
                for sensortagservice in SensorTagServices:
                    try:
                        if service.uuid == self._UUIDH2B16(sensortagservice.value) or service.uuid == self._UUIDH2B2(sensortagservice.value):
                            self.services[sensortagservice.name] = {}
                            self.services[sensortagservice.name]['Service'] = service
                    except:
                        pass
                self.connection.find_information(service, timeout = 10)
                try:
                    self.connection.read_by_type(service, bgapi.module.GATTCharacteristic.CHARACTERISTIC_UUID, timeout = 10)
                    self.connection.read_by_type(service, bgapi.module.GATTCharacteristic.CLIENT_CHARACTERISTIC_CONFIG, timeout = 10)
                except:
                    pass
            characteristics = self.connection.get_characteristics()
            for chr in characteristics:
                for schr in DeviceCharacteristics:
                    if chr.uuid == self._UUIDH2B2(schr.value):
                        self.services['Device'][schr.name] = chr
                for schr in DeviceInformationCharacteristics:
                    if chr.uuid == self._UUIDH2B2(schr.value):
                        self.services['DeviceInformation'][schr.name] = chr
                for schr in BatteryCharacteristics:
                    if chr.uuid == self._UUIDH2B2(schr.value):
                        self.services['Battery'][schr.name] = chr
                for schr in ButtonCharacteristics:
                    if chr.uuid == self._UUIDH2B2(schr.value):
                        self.services['Button'][schr.name] = chr
                for schr in IRTempCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['IRTemp'][schr.name] = chr
                for schr in HumidityCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['Humidity'][schr.name] = chr
                for schr in BarometerCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['Barometer'][schr.name] = chr
                for schr in IOCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['IO'][schr.name] = chr
                for schr in LuxometerCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['Luxometer'][schr.name] = chr
                for schr in MovementCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['Movement'][schr.name] = chr
                for schr in RegisterCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['Register'][schr.name] = chr
                for schr in DisplayCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['Display'][schr.name] = chr
                for schr in ConnectionControlCharacteristics:
                    if chr.uuid == self._UUIDH2B16(schr.value):
                        self.services['ConnectionControl'][schr.name] = chr

    def disconnect(self):
        if self.connected:
            self.connection.set_disconnected(0)
            self.client.reset_ble_state()

    def twotoone(self, b0, b1):
        return (b0 & 0xFF) | ((b1 & 0xFF) << 8)
    
    def threetoone(self, b0, b1, b2):
        return (b0 & 0xFF) | ((b1 & 0xFF) << 8) | ((b2 & 0xFF) << 16)

    def fourtoone(self, b0, b1, b2, b3):
        return (b0 & 0xFF) | ((b1 & 0xFF) << 8) | ((b2 & 0xFF) << 16) | ((b3 & 0xFF) << 24)

    def u8tos8(self, value):
        return ((value & 0xFF) ^ (1 << 7)) - (1 << 7)

    def u16tos16(self, value):
        return ((value & 0xFFFF) ^ (1 << 15)) - (1 << 15)
    
    def u24tos24(self, value):
        return ((value & 0xFFFFFF) ^ (1 << 23)) - (1 << 23)
    
    def u32tos32(self, value):
        return ((value & 0xFFFFFFFF) ^ (1 << 31)) - (1 << 31)
    
    def ltostr(self, value):
        return bytes(value).decode('ASCII')

    def readValue(self, sensor: str, property: str) -> list:
        if self.connected:
            attempts = 0
            while True:
                try:
                    self.connection.read_by_handle(self.services[sensor][property].value_handle)
                    break
                except:
                    attempts += 1
                    if attempts > self.maximumAttempts:
                        raise TimeoutError("Maximum attempts exceeded")
                    time.sleep(0.001)
                    pass
            time.sleep(0.2)
            return list(self.services[sensor][property].value)
    
    def writeValue(self, sensor: str, property: str, value: list):
        if self.connected:
            attempts = 0
            while True:
                try:
                    self.connection.write_by_handle(self.services[sensor][property].value_handle, bytes(value))
                    break
                except:
                    attempts += 1
                    if attempts > self.maximumAttempts:
                        raise TimeoutError("Maximum attempts exceeded")
                    time.sleep(0.001)
                    pass
            time.sleep(0.2)
            return

    def assignCallback(self, sensor: str, property: str, callback, filter = lambda v : v):
        if self.connected:
            chr = self.services[sensor][property]
            if chr.has_notify() or chr.has_indicate():
                self.connection.assign_attrclient_value_callback(chr.value_handle, lambda c : callback(sensor, filter(list(c))))
                self.connection.characteristic_subscription(chr, chr.has_indicate(), chr.has_notify() and not chr.has_indicate())

    def removeCallback(self, sensor: str, property: str):
        if self.connected:
            chr = self.services[sensor][property]
            if chr.has_notify() or chr.has_indicate():
                self.connection.characteristic_subscription(chr, False, False)

    # Device Methods
    def readDeviceNameRaw(self):
        if self.connected:
            return self.readValue('Device', 'Name')
    
    def convertDeviceNameStr(self, raw: list):
        return self.ltostr(raw)
    
    def readDeviceAppearanceRaw(self):
        if self.connected:
            return self.readValue('Device', 'Appearance')
    
    def convertDeviceAppearance(self, raw):
        return self.twotoone(raw[0], raw[1])
    
    def readDevicePreferredConnectionParametersRaw(self):
        if self.connected:
            return self.readValue('Device', 'PreferredConnectionParameters')
    
    def convertDevicePreferredConnectionParameters(self, raw):
        result = {}
        result['MinConnectionInterval'] = self.twotoone(raw[DevicePreferredConnectionParameters.MinConnectionIntervalLSB], raw[DevicePreferredConnectionParameters.MinConnectionIntervalMSB])
        result['MaxConnectionInterval'] = self.twotoone(raw[DevicePreferredConnectionParameters.MaxConnectionIntervalLSB], raw[DevicePreferredConnectionParameters.MaxConnectionIntervalMSB])
        result['SlaveLatency'] = self.twotoone(raw[DevicePreferredConnectionParameters.SlaveLatencyLSB], raw[DevicePreferredConnectionParameters.SlaveLatencyMSB])
        result['SupervisionTimeout'] = self.twotoone(raw[DevicePreferredConnectionParameters.SupervisionTimeoutLSB], raw[DevicePreferredConnectionParameters.SupervisionTimeoutMSB])
        return result
    # Device Methods

    # DeviceInformationMethods
    def readDeviceInformationSystemIDRaw(self):
        if self.connected:
            return self.readValue('DeviceInformation', 'SystemID')

    def readDeviceInformationModelNumberStr(self):
        if self.connected:
            return self.ltostr(self.readValue('DeviceInformation', 'ModelNumber'))

    def readDeviceInformationSerialNumberStr(self):
        if self.connected:
            return self.ltostr(self.readValue('DeviceInformation', 'SerialNumber'))

    def readDeviceInformationFirmwareRevisionStr(self):
        if self.connected:
            return self.ltostr(self.readValue('DeviceInformation', 'FirmwareRevision'))

    def readDeviceInformationHardwareRevisionStr(self):
        if self.connected:
            return self.ltostr(self.readValue('DeviceInformation', 'HardwareRevision'))

    def readDeviceInformationSoftwareRevisionStr(self):
        if self.connected:
            return self.ltostr(self.readValue('DeviceInformation', 'SoftwareRevision'))

    def readDeviceInformationManufacturerNameStr(self):
        if self.connected:
            return self.ltostr(self.readValue('DeviceInformation', 'ManufacturerName'))

    def readDeviceInformationPnPIDRaw(self):
        if self.connected:
            return self.readValue('DeviceInformation', 'PnPID')

    def convertDeviceInformationPnPID(self, raw):
        result = {}
        result['CompanyIdentifier'] = self.twotoone(raw[DeviceInformationPnPID.CompanyLSB], raw[DeviceInformationPnPID.CompanyMSB])
        result['ProductID'] = self.twotoone(raw[DeviceInformationPnPID.ProductIDLSB], raw[DeviceInformationPnPID.ProductIDMSB])
        result['ProductVersion'] = self.twotoone(raw[DeviceInformationPnPID.ProductVersionLSB], raw[DeviceInformationPnPID.ProductVersionMSB])
        return result
    # DeviceInformationMethods
    
    # Battery Methods
    def readBatteryLevelRaw(self):
        if self.connected:
            return self.readValue('Battery', 'BatteryLevel')
        
    def convertBatteryLevelPercent(self, raw):
        return raw[0]
    # Battery Methods

    # Button Methods
    def convertButtonData(self, raw):
        result = {}
        result['UserButton'] = (raw[0] & ButtonData.UserButtonMask) >> ButtonData.UserButtonShift
        result['PowerButton'] = (raw[0] & ButtonData.PowerButtonMask) >> ButtonData.PowerButtonShift
        result['ReedRelay'] = (raw[0] & ButtonData.ReedRelayMask) >> ButtonData.ReedRelayShift
        return result
    # Button Methods

    # IRTemp Methods
    def readIRTempDataRaw(self):
        if self.connected:
            return self.readValue('IRTemp', 'Data')
    
    def convertIRTempObjectRaw(self, raw):
        return self.twotoone(raw[IRTempData.ObjectLSB], raw[IRTempData.ObjectMSB])

    def convertIRTempObjectCelcius(self, raw):
        return self.convertIRTempObjectRaw(raw) / 128
    
    def convertIRTempAmbianceRaw(self, raw):
        return self.twotoone(raw[IRTempData.AmbienceLSB], raw[IRTempData.AmbienceMSB])
    
    def convertIRTempAmbianceCelcius(self, raw):
        return self.convertIRTempAmbianceRaw(raw) / 128
    
    def readIRTempConfiguration(self):
        if self.connected:
            return IRTempConfiguration(self.readValue('IRTemp', 'Configuration')[0])
    
    def enableIRTemp(self):
        if self.connected:
            self.writeValue('IRTemp', 'Configuration', [IRTempConfiguration.Enable.value])
    
    def disableIRTemp(self):
        if self.connected:
            self.writeValue('IRTemp', 'Configuration', [IRTempConfiguration.Disable.value])
    
    def readIRTempPeriodRaw(self):
        if self.connected:
            return self.readValue('IRTemp', 'Period')
    
    def convertIRTempPeriodMS(self, raw):
        return raw[0] * 10
    
    def changeIRTempPeriod(self, ms: int):
        if self.connected:
            ms = int(ms / 10)
            self.writeValue('IRTemp', 'Period', [min(IRTempPeriod.Max, max(IRTempPeriod.Min, ms))])
    # IRTemp Methods

    # Humidity Methods
    def readHumidityDataRaw(self):
        if self.connected:
            return self.readValue('Humidity', 'Data')
    
    def convertHumidityTempRaw(self, raw):
        return self.twotoone(raw[HumidityData.TempLSB], raw[HumidityData.TempMSB])
    
    def convertHumidityTempCelcius(self, raw):
        return ((self.convertHumidityTempRaw(raw) / 65536) * 165) - 40

    def convertHumidityHumRaw(self, raw):
        return self.twotoone(raw[HumidityData.HumLSB], raw[HumidityData.HumMSB])
    
    def convertHumidityHumRH(self, raw):
        return (self.convertHumidityHumRaw(raw) / 65536) * 100
    
    def readHumidityConfiguration(self):
        if self.connected:
            return HumidityConfiguration(self.readValue('Humidity', 'Configuration')[0])
    
    def enableHumidity(self):
        if self.connected:
            self.writeValue('Humidity', 'Configuration', [HumidityConfiguration.Enable.value])
    
    def disableHumidity(self):
        if self.connected:
            self.writeValue('Humidity', 'Configuration', [HumidityConfiguration.Disable.value])
    
    def readHumidityPeriodRaw(self):
        if self.connected:
            return self.readValue('Humidity', 'Period')
    
    def convertHumidityPeriodMS(self, raw):
        return raw[0] * 10
    
    def changeHumidityPeriod(self, ms: int):
        if self.connected:
            ms = int(ms / 10)
            self.writeValue('Humidity', 'Period', [min(HumidityPeriod.Max, max(HumidityPeriod.Min, ms))])
    # Humidity Methods

    # Barometer Methods
    def readBarometerDataRaw(self):
        if self.connected:
            return self.readValue('Barometer', 'Data')
    
    def convertBarometerTempRaw(self, raw):
        return self.threetoone(raw[BarometerData.TempB0], raw[BarometerData.TempB1], raw[BarometerData.TempB2])
    
    def convertBarometerTempCelcius(self, raw):
        return self.convertBarometerTempRaw(raw) / 100

    def convertBarometerPressRaw(self, raw):
        return self.threetoone(raw[BarometerData.PressB0], raw[BarometerData.PressB1], raw[BarometerData.PressB2])

    def convertBarometerPressHPa(self, raw):
        return self.convertBarometerPressRaw(raw) / 100
    
    def readBarometerConfiguration(self):
        if self.connected:
            return BarometerConfiguration(self.readValue('Barometer', 'Configuration')[0])
    
    def enableBarometer(self):
        if self.connected:
            self.writeValue('Barometer', 'Configuration', [BarometerConfiguration.Enable.value])
    
    def disableBarometer(self):
        if self.connected:
            self.writeValue('Barometer', 'Configuration', [BarometerConfiguration.Disable.value])
    
    def readBarometerPeriodRaw(self):
        if self.connected:
            return self.readValue('Barometer', 'Period')
    
    def convertBarometerPeriodMS(self, raw):
        return raw[0] * 10
    
    def changeBarometerPeriod(self, ms: int):
        if self.connected:
            ms = int(ms / 10)
            self.writeValue('Barometer', 'Period', [min(BarometerPeriod.Max, max(BarometerPeriod.Min, ms))])
    # Barometer Methods

    # IO Methods
    def readIODataRaw(self):
        if self.connected:
            return self.readValue('IO', 'Data')[0]
    
    def convertIOLocalData(self, raw):
        return self.readIODataRaw()
    
    def convertIORemoteData(self, raw):
        value = self.readIODataRaw()
        result = {}
        result['Green'] = bool(value & IODataRemote.GreenMask)
        result['Red'] = bool(value & IODataRemote.RedMask)
        result['Buzzer'] = bool(value & IODataRemote.BuzzerMask)
        return result

    def convertIOTestResult(self, raw):
        value = self.readIODataRaw()
        result = {}
        result['IRTemp'] = bool(value & IODataTest.IRTempMask)
        result['Humidity'] = bool(value & IODataTest.HumidityMask)
        result['Luxometer'] = bool(value & IODataTest.LuxometerMask)
        result['Barometer'] = bool(value & IODataTest.BarometerMask)
        result['Movement'] = bool(value & IODataTest.MovementMask)
        result['Magnetometer'] = bool(value & IODataTest.MagnetometerMask)
        result['ExternalFlash'] = bool(value & IODataTest.ExternalFlashMask)
        result['DevPack'] = bool(value & IODataTest.DevPackMask)
        return result
    
    def setIORedLed(self, state: bool):
        if self.connected:
            value = self.readIODataRaw()
            if state:
                value = value | (0xFF & IODataRemote.RedMask)
            else:
                value = value & (0xFF ^ IODataRemote.RedMask)
            self.writeValue('IO', 'Data', [value])
    
    def setIOGreenLed(self, state: bool):
        if self.connected:
            value = self.readIODataRaw()
            if state:
                value = value | (0xFF & IODataRemote.GreenMask)
            else:
                value = value & (0xFF ^ IODataRemote.GreenMask)
            self.writeValue('IO', 'Data', [value])
    
    def setIOBuzzer(self, state: bool):
        if self.connected:
            value = self.readIODataRaw()
            if state:
                value = value | (0xFF & IODataRemote.BuzzerMask)
            else:
                value = value & (0xFF ^ IODataRemote.BuzzerMask)
            self.writeValue('IO', 'Data', [value])

    def readIOMode(self):
        if self.connected:
            return IOMode(self.readValue('IO', 'Configuration')[0])
    
    def changeIOMode(self, mode: IOMode):
        if self.connected:
            if mode == IOMode.Remote:
                self.writeValue('IO', 'Data', [0])
                time.sleep(0.1)
            self.writeValue('IO', 'Configuration', [mode.value])
    # IO Methods

    # Luxometer Methods
    def readLuxometerDataRaw(self):
        if self.connected:
            return self.readValue('Luxometer', 'Data')
    
    def convertLuxometerLightRaw(self, raw):
        return self.twotoone(raw[LuxometerData.LightLSB], raw[LuxometerData.LightMSB])
    
    def convertLuxometerLightLux(self, raw):
        value = self.convertLuxometerLightRaw(raw)
        m = value & 0x0FFF
        e = (value & 0xF000) >> 12
        return m * (0.01 * (2 ** e))
    
    def readLuxometerConfiguration(self):
        if self.connected:
            return LuxometerConfiguration(self.readValue('Luxometer', 'Configuration')[0])
    
    def enableLuxometer(self):
        if self.connected:
            self.writeValue('Luxometer', 'Configuration', [LuxometerConfiguration.Enable.value])
    
    def disableLuxometer(self):
        if self.connected:
            self.writeValue('Luxometer', 'Configuration', [LuxometerConfiguration.Disable.value])
    
    def readLuxometerPeriodRaw(self):
        if self.connected:
            return self.readValue('Luxometer', 'Period')
    
    def convertLuxometerPeriodMS(self, raw):
        return raw[0] * 10
    
    def changeLuxometerPeriod(self, ms: int):
        if self.connected:
            ms = int(ms / 10)
            self.writeValue('Luxometer', 'Period', [min(LuxometerPeriod.Max, max(LuxometerPeriod.Min, ms))])
    # Luxometer Methods

    # Movement Methods
    def readMovementDataRaw(self):
        if self.connected:
            return self.readValue('Movement', 'Data')
    
    def convertMovementDataRaw(self, raw):
        result = {}
        result['Gyroscope'] = self.convertMovementGyroRaw(raw)
        result['Accelerometer'] = self.convertMovementAccRaw(raw)
        result['Magnetometer'] = self.convertMovementMagRaw(raw)
        return result
    
    def convertMovementDataUnit(self, raw):
        result = {}
        result['Gyroscope'] = self.convertMovementGyroRot(raw)
        result['Accelerometer'] = self.convertMovementAccG(raw)
        result['Magnetometer'] = self.convertMovementMaguT(raw)
        return result

    def convertMovementGyroRaw(self, raw):
        x = self.u16tos16(self.twotoone(raw[MovementData.GyroXLSB], raw[MovementData.GyroXMSB]))
        y = self.u16tos16(self.twotoone(raw[MovementData.GyroYLSB], raw[MovementData.GyroYMSB]))
        z = self.u16tos16(self.twotoone(raw[MovementData.GyroZLSB], raw[MovementData.GyroZMSB]))
        return (x, y, z)
    
    def convertMovementGyroRot(self, raw):
        x, y, z = self.convertMovementGyroRaw(raw)
        x = x / (65536 / 500)
        y = y / (65536 / 500)
        z = z / (65536 / 500)
        return (x, y, z)

    def convertMovementAccRaw(self, raw):
        x = self.u16tos16(self.twotoone(raw[MovementData.AccXLSB], raw[MovementData.AccXMSB]))
        y = self.u16tos16(self.twotoone(raw[MovementData.AccYLSB], raw[MovementData.AccYMSB]))
        z = self.u16tos16(self.twotoone(raw[MovementData.AccZLSB], raw[MovementData.AccZMSB]))
        return (x, y, z)
    
    def convertMovementAccG(self, raw):
        x, y, z = self.convertMovementAccRaw(raw)
        x = x / 32768
        y = y / 32768
        z = z / 32768
        return (x, y, z)
    
    def applyMovementAccGRange(self, g, range):
        x, y, z = g
        x *= range
        y *= range
        z *= range
        return (x, y, z)
    
    def convertMovementMagRaw(self, raw):
        x = self.u16tos16(self.twotoone(raw[MovementData.MagXLSB], raw[MovementData.MagXMSB]))
        y = self.u16tos16(self.twotoone(raw[MovementData.MagYLSB], raw[MovementData.MagYMSB]))
        z = self.u16tos16(self.twotoone(raw[MovementData.MagZLSB], raw[MovementData.MagZMSB]))
        return (x, y, z)
    
    def convertMovementMaguT(self, raw):
        return self.convertMovementMagRaw(raw)
    
    def readMovementConfigurationRaw(self):
        if self.connected:
            return self.readValue('Movement', 'Configuration')

    def convertMovementConfiguration(self, raw):
        value = self.twotoone(raw[MovementConfiguration.LSB], raw[MovementConfiguration.MSB])
        result = {}
        result['GyroZEnable'] = bool(value & MovementConfiguration.GyroZMask)
        result['GyroYEnable'] = bool(value & MovementConfiguration.GyroYMask)
        result['GyroXEnable'] = bool(value & MovementConfiguration.GyroXMask)
        result['AccZEnable'] = bool(value & MovementConfiguration.AccZMask)
        result['AccYEnable'] = bool(value & MovementConfiguration.AccYMask)
        result['AccXEnable'] = bool(value & MovementConfiguration.AccXMask)
        result['MagnetometerEnable'] = bool(value & MovementConfiguration.MagMask)
        result['WakeOnMotionEnable'] = bool(value & MovementConfiguration.WakeOnMotionMask)
        result['AccRange'] = AccelerometerRange((value & MovementConfiguration.AccRangeMask) >> MovementConfiguration.AccRangeShift)
        return result

    def enableMovementGyro(self, x: bool, y: bool, z: bool):
        if self.connected:
            raw = self.readMovementConfigurationRaw()
            value = self.twotoone(raw[MovementConfiguration.LSB], raw[MovementConfiguration.MSB])
            if x:
                value = value | (0xFFFF & MovementConfiguration.GyroXMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.GyroXMask)
            if y:
                value = value | (0xFFFF & MovementConfiguration.GyroYMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.GyroYMask)
            if z:
                value = value | (0xFFFF & MovementConfiguration.GyroZMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.GyroZMask)
            self.writeValue('Movement', 'Configuration', [value & 0xFF, (value >> 8) & 0xFF])
    
    def enableMovementAcc(self, x: bool, y: bool, z: bool):
        if self.connected:
            raw = self.readMovementConfigurationRaw()
            value = self.twotoone(raw[MovementConfiguration.LSB], raw[MovementConfiguration.MSB])
            if x:
                value = value | (0xFFFF & MovementConfiguration.AccXMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.AccXMask)
            if y:
                value = value | (0xFFFF & MovementConfiguration.AccYMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.AccYMask)
            if z:
                value = value | (0xFFFF & MovementConfiguration.AccZMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.AccZMask)
            self.writeValue('Movement', 'Configuration', [value & 0xFF, (value >> 8) & 0xFF])
    
    def enableMovementMag(self, state: bool):
        if self.connected:
            raw = self.readMovementConfigurationRaw()
            value = self.twotoone(raw[MovementConfiguration.LSB], raw[MovementConfiguration.MSB])
            if state:
                value = value | (0xFFFF & MovementConfiguration.MagMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.MagMask)
            self.writeValue('Movement', 'Configuration', [value & 0xFF, (value >> 8) & 0xFF])

    def enableMovementWakeOnMotion(self, state: bool):
        if self.connected:
            raw = self.readMovementConfigurationRaw()
            value = self.twotoone(raw[MovementConfiguration.LSB], raw[MovementConfiguration.MSB])
            if state:
                value = value | (0xFFFF & MovementConfiguration.WakeOnMotionMask)
            else:
                value = value & (0xFFFF ^ MovementConfiguration.WakeOnMotionMask)
            self.writeValue('Movement', 'Configuration', [value & 0xFF, (value >> 8) & 0xFF])
    
    def changeMovementAccRange(self, g: AccelerometerRange):
        if self.connected:
            raw = self.readMovementConfigurationRaw()
            value = self.twotoone(raw[MovementConfiguration.LSB], raw[MovementConfiguration.MSB])
            value = value & (0xFFFF ^ (0b11 << MovementConfiguration.AccRangeShift))
            value = value | (0xFFFF & (g.value << MovementConfiguration.AccRangeShift))
            self.writeValue('Movement', 'Configuration', [value & 0xFF, (value >> 8) & 0xFF])
    
    def readMovementPeriodRaw(self):
        if self.connected:
            return self.readValue('Movement', 'Period')
    
    def convertMovementPeriodMS(self, raw):
        return raw[0] * 10
    
    def changeMovementPeriod(self, ms: int):
        if self.connected:
            ms = int(ms / 10)
            self.writeValue('Movement', 'Period', [min(MovementPeriod.Max, max(MovementPeriod.Min, ms))])
    # Movement Methods

    # Register Methods
    def readRegisterDataRaw(self):
        if self.connected:
            return self.readValue('Register', 'Data')
    
    def convertRegisterData(self, raw):
        if len(raw) == 1:
            return raw[0]
        elif len(raw) == 2:
            return self.twotoone(raw[0], raw[1])
        elif len(raw) == 3:
            return self.threetoone(raw[0], raw[1], raw[2])
        elif len(raw) == 4:
            return self.fourtoone(raw[0], raw[1], raw[2], raw[3])

    def writeRegisterData(self, data):
        if self.connected:
            self.writeValue('Register', 'Data', [data & 0xFF,
                                                (data >> 8) & 0xFF,
                                                (data >> 16) & 0xFF,
                                                (data >> 24) & 0xFF])

    def readRegisterAddressRaw(self):
        if self.connected:
            return self.readValue('Register', 'Address')
    
    def convertRegisterAddress(self, raw):
        if len(raw) == 2:
            return raw[0], raw[1]
        elif len(raw) == 3:
            return raw[0], self.twotoone(raw[1], raw[2])
        elif len(raw) == 4:
            return raw[0], self.threetoone(raw[1], raw[2], raw[3])
        elif len(raw) == 5:
            return raw[0], self.fourtoone(raw[1], raw[2], raw[3], raw[4])
    
    def changeRegisterAddress(self, size, address):
        if self.connected:
            if address < 0x100:
                self.writeValue('Register', 'Address', [size, address])
            elif address < 0x10000:
                self.writeValue('Register', 'Address', [size, address & 0xFF, 
                                                        (address >> 8) & 0xFF])
            elif address < 0x1000000:
                self.writeValue('Register', 'Address', [size, address & 0xFF, 
                                                        (address >> 8) & 0xFF, 
                                                        (address >> 16) & 0xFF])
            elif address < 0x100000000:
                self.writeValue('Register', 'Address', [size, address & 0xFF, 
                                                        (address >> 8) & 0xFF, 
                                                        (address >> 16) & 0xFF,
                                                        (address >> 24) & 0xFF])

    def readRegisterDeviceIDRaw(self):
        if self.connected:
            return self.readValue('Register', 'DeviceID')
    
    def convertRegisterDeviceID(self, raw):
        for device in RegisterAddresses:
            if raw == device.value:
                return device
    
    def selectRegisterDeviceIRTemp(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', list(RegisterAddresses.IRTemp.value))
    
    def selectRegisterDeviceHumidity(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', list(RegisterAddresses.Humidity.value))
    
    def selectRegisterDeviceBarometer(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', list(RegisterAddresses.Barometer.value))
    
    def selectRegisterDeviceLuxometer(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', list(RegisterAddresses.Luxometer.value))
    
    def selectRegisterDeviceMovement(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', list(RegisterAddresses.Magnetometer.value))

    def selectRegisterDeviceMovement(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', list(RegisterAddresses.Movement.value))
    
    def selectRegisterInterfaceI2C0(self, id: int):
        if self.connected:
            self.writeValue('Register', 'DeviceID', [RegisterInterfaceIDs.I2C0, id])

    def selectRegisterInterfaceI2C1(self, id: int):
        if self.connected:
            self.writeValue('Register', 'DeviceID', [RegisterInterfaceIDs.I2C1, id])
    
    def selectRegisterInterfaceSPI(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', [RegisterInterfaceIDs.SPI, 0])

    def selectRegisterInterfaceMCU(self):
        if self.connected:
            self.writeValue('Register', 'DeviceID', [RegisterInterfaceIDs.MCU, 0])
    # Register Methods

    # Display Methods
    def readDisplayDataRaw(self):
        if self.connected:
            return self.readValue('Display', 'Data')
    
    def convertDisplayDataStr(self, raw):
        return str(raw, encoding = 'ASCII')
    
    def writeDisplayText(self, text: str, row: int = None, column: int = None, clear: bool = False, wrap: bool = False):
        if self.connected:
            splitted = []
            text = text.strip()
            if row != None or column != None:
                row = row % DisplayLimit.Row if row else 0
                column = column % DisplayLimit.Column if column else 0
                if wrap:
                    column = 0
                    splitted = []
                    for i in range(int(len(text) / DisplayLimit.Column) + 1):
                        try:
                            splitted.append(text[i * DisplayLimit.Column:(i + 1) * DisplayLimit.Column])
                        except:
                            splitted.append(text[i * DisplayLimit.Column:])
                else:
                    splitted.append(text)
                for t in splitted:
                    if clear:
                        self.setDisplayClearRow(row)
                    self.setDisplayCursor(row, column)
                    self.writeValue('Display', 'Data', bytes(t, 'ASCII'))
                    row = (row + 1) % DisplayLimit.Row
            else:
                self.writeValue('Display', 'Data', bytes(text, 'ASCII') if len(text) < 16 else bytes(text[:16], 'ASCII'))

    def setDisplayOff(self):
        if self.connected:
            self.writeValue('Display', 'Control', [DisplayCommands.DisplayOff])
    
    def setDisplayOn(self):
        if self.connected:
            self.writeValue('Display', 'Control', [DisplayCommands.DisplayOn])
    
    def setDisplayClearAll(self):
        if self.connected:
            self.writeValue('Display', 'Control', [DisplayCommands.ClearAll])
    
    def setDisplayClearRow(self, row: int):
        if self.connected:
            self.writeValue('Display', 'Control', [DisplayCommands.ClearLine, row])
    
    def setDisplayInvertColor(self):
        if self.connected:
            self.writeValue('Display', 'Control', [DisplayCommands.InvertColor])
    
    def setDisplayCursor(self, row: int, column: int):
        if self.connected:
            self.writeValue('Display', 'Control', [DisplayCommands.SetPosition, row, column])
    # Display Methods
    
    # Connection Control Methods
    def readConnectionParametersRaw(self):
        if self.connected:
            return self.readValue('ConnectionControl', 'ConnectionParameters')
    
    def convertConnectionParameters(self, raw):
        result = {}
        result['ConnectionInterval'] = self.twotoone(raw[ConnectionControlData.ConnectionIntervalLSB],
                                                     raw[ConnectionControlData.ConnectionIntervalMSB])
        result['SlaveLatency'] = self.twotoone(raw[ConnectionControlData.SlaveLatencyLSB],
                                               raw[ConnectionControlData.SlaveLatencyMSB])
        result['SupervisionTimeout'] = self.twotoone(raw[ConnectionControlData.SupervisionTimeoutLSB],
                                                     raw[ConnectionControlData.SupervisionTimeoutMSB])
        return result

    def requestConnectionParameters(self, maxConnectionInterval, minConnectionInterval, slaveLatency, supervisionTimeout):
        if self.connected:
            self.writeValue('ConnectionControl', 'RequestConnectionParameters', 
            [maxConnectionInterval & 0xFF,
            (maxConnectionInterval >> 8) & 0xFF,
            minConnectionInterval & 0xFF,
            (minConnectionInterval >> 8) & 0xFF,
            slaveLatency & 0xFF,
            (slaveLatency >> 8) & 0xFF,
            supervisionTimeout & 0xFF,
            (supervisionTimeout >> 8) & 0xFF])
    
    def requestConnectionDisconnect(self):
        if self.connected:
            self.writeValue('ConnectionControl', 'RequestDisconnect', [1])
    # Connection Control Methods