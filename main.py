# ============================
# main.py — Full Sensor System
# ============================

from machine import Pin, SoftI2C
from time import sleep, localtime, ticks_ms, ticks_diff
import urequests
import time


print("main.py running...")

# -------------------------------------------------------
# 1) ThingSpeak API Keys
# -------------------------------------------------------
API_A = "EU6EE36IJ7WSVYP3"
API_B = "E8CTAK8MCUWLVQJ2"
API_C = "Y1FWSOX7Z6YZ8QMU"
API_W = "HG8GG8DF40LCGV99"


# -------------------------------------------------------
# 2) ThingSpeak POST function
# -------------------------------------------------------
def send_to_thingspeak(api_key, fields):
    url = "https://api.thingspeak.com/update"
    payload = "api_key=" + api_key
    for i, v in enumerate(fields, start=1):
        if v is None:
            v = 0
        payload += "&field{}={}".format(i, v)
    try:
        r = urequests.post(url, data=payload)
        r.close()
        print("→ TS:", api_key[:6], fields)
    except Exception as e:
        print("TS error:", e)


# -------------------------------------------------------
# 3) Configure I2C buses for A / B / C models
# -------------------------------------------------------
i2c_A1 = SoftI2C(sda=Pin(19), scl=Pin(18), freq=100000)
i2c_A2 = SoftI2C(sda=Pin(23), scl=Pin(5),  freq=100000)

i2c_B1 = SoftI2C(sda=Pin(25), scl=Pin(26), freq=100000)
i2c_B2 = SoftI2C(sda=Pin(27), scl=Pin(14), freq=100000)

i2c_C1 = SoftI2C(sda=Pin(32), scl=Pin(0),  freq=25000)
i2c_C2 = SoftI2C(sda=Pin(15), scl=Pin(2),  freq=25000)

print("I2C buses ready.")

# -------------------------------------------------------
# 4) SHT30 Temp & Hum sensors
# -------------------------------------------------------
from lib.sht30 import SHT30

sht_air_A = SHT30(i2c_A2, addr=0x45)
sht_w2_A  = SHT30(i2c_A2, addr=0x44)
sht_amb   = SHT30(i2c_A1, addr=0x44)

sht_air_B = SHT30(i2c_B2, addr=0x45)
sht_w2_B  = SHT30(i2c_B2, addr=0x44)

sht_air_C = SHT30(i2c_C2, addr=0x45)
sht_w2_C  = SHT30(i2c_C2, addr=0x44)


def safe_read_sht(sht):
    try:
        t, rh = sht.measure()
        return t, rh
    except:
        return None, None


# -------------------------------------------------------
# 5) HX711 Mass sensors
# -------------------------------------------------------
from lib.hx711 import HX711

hxA = HX711(34, 33); hxA.set_scale(1.0)
hxB = HX711(35, 33); hxB.set_scale(1.0)
hxC = HX711(36, 33); hxC.set_scale(1.0)
try:
    hxA.tare(); hxB.tare(); hxC.tare()
except:
    pass


def safe_mass(hx):
    try:
        return hx.get_units(5)
    except:
        return None


# -------------------------------------------------------
# 6) LTR390 UV Sensor
# -------------------------------------------------------
LTR_ADDR = 0x53
MAIN_CTRL = 0x00
ALS_UVS_MEAS_RATE = 0x04
ALS_UVS_GAIN = 0x05
MAIN_STATUS = 0x07
UVS_DATA_0 = 0x10

def _w8(i2c, r, v): i2c.writeto_mem(LTR_ADDR, r, bytes([v & 0xFF]))
def _r8(i2c, r): return i2c.readfrom_mem(LTR_ADDR, r, 1)[0]

def _r20(i2c, base):
    b0 = _r8(i2c, base)
    b1 = _r8(i2c, base+1)
    b2 = _r8(i2c, base+2)
    return ((b2 << 16) | (b1 << 8) | b0) & 0xFFFFF

def _ltr_ready(i2c, to_ms=300):
    t0 = time.ticks_ms()
    while True:
        try:
            if _r8(i2c, MAIN_STATUS) & 0x08:
                return True
        except:
            pass
        if time.ticks_diff(time.ticks_ms(), t0) > to_ms:
            return False
        time.sleep_ms(2)

def ltr_init_uv(i2c, gain=3, it_ms=100):
    GAIN_MAP = {1:0, 3:1, 6:2, 9:3, 18:4}
    ITS = [25,50,100,200,400,800]
    it = min(ITS, key=lambda x: abs(x-it_ms))
    RES_BITS  = {25:3,50:3,100:3,200:4,400:5,800:5}
    RATE_BITS = {25:0,50:1,100:2,200:3,400:4,800:5}
    _w8(i2c, ALS_UVS_GAIN, GAIN_MAP.get(gain,1))
    _w8(i2c, ALS_UVS_MEAS_RATE, (RATE_BITS[it]<<3) | (RES_BITS[it]&7))
    _w8(i2c, MAIN_CTRL, 0x06)

def ltr_read_uv(i2c, uvi_factor=1/2300):
    try:
        _w8(i2c, MAIN_CTRL, 0x06)
        if not _ltr_ready(i2c):
            return None, None
        uvs = _r20(i2c, UVS_DATA_0)
        uvi = uvs * uvi_factor if uvs else None
        return uvs, uvi
    except:
        return None, None

for bus in (i2c_A1, i2c_B1, i2c_C1):
    try:
        ltr_init_uv(bus)
    except:
        pass


# -------------------------------------------------------
# 7) TSL2591 Light Sensor (IR + Lux)
# -------------------------------------------------------
class TSL2591:
    I2C_ADDR = 0x29
    CMD = 0xA0
    ENABLE = 0x00
    CONTROL = 0x01
    C0DATAL = 0x14
    C0DATAH = 0x15
    C1DATAL = 0x16
    C1DATAH = 0x17
    PON = 0x01
    AEN = 0x02
    _ATIME_CODE={100:0x00,200:0x01,300:0x02,400:0x03,500:0x04,600:0x05}
    _AGAIN_CODE={1:0x00,25:0x10,428:0x20,9876:0x30}

    def __init__(self, i2c, it_ms=400, gain=25):
        self.i2c=i2c; self.it_ms=it_ms; self.gain=gain
        self._w8(self.ENABLE, self.PON)
        sleep(0.01)
        self._w8(self.CONTROL, self._AGAIN_CODE[gain] | self._ATIME_CODE[it_ms])
        self._w8(self.ENABLE, self.PON | self.AEN)

    def _w8(self,r,v):
        self.i2c.writeto_mem(self.I2C_ADDR, self.CMD | r, bytes([v]))

    def _r8(self,r):
        return self.i2c.readfrom_mem(self.I2C_ADDR, self.CMD | r, 1)[0]

    def _r16(self,L,H):
        lo=self._r8(L); hi=self._r8(H)
        return (hi<<8)|lo

    def read_ir_lux(self):
        self._w8(self.ENABLE, self.PON | self.AEN)
        sleep(self.it_ms/1000)
        full=self._r16(self.C0DATAL,self.C0DATAH)
        ir  =self._r16(self.C1DATAL,self.C1DATAH)
        vis=max(0,full-ir)
        cpl=(self.it_ms*self.gain)/408.0
        lux=(vis/cpl) if cpl>0 else None
        return ir,lux

tsl_A = TSL2591(i2c_A2)
tsl_B = TSL2591(i2c_B2)
tsl_C = TSL2591(i2c_C2)

def safe_tsl(tsl):
    try:
        return tsl.read_ir_lux()
    except:
        return None, None


# -------------------------------------------------------
# 8) VL53L0X Distance Sensor
# -------------------------------------------------------
SYSRANGE_START = 0x00
RANGE_MM_HI    = 0x1E
RANGE_MM_LO    = 0x1F

def _vl_w8(i2c, addr, reg, v):
    i2c.writeto_mem(addr, reg, bytes([v]))

def _vl_r8(i2c, addr, reg):
    return i2c.readfrom_mem(addr, reg, 1)[0]

def vl_once_raw(i2c, addr=0x29, delay_ms=60):
    try:
        _vl_w8(i2c, addr, SYSRANGE_START, 0x01)
        sleep(delay_ms/1000)
        hi=_vl_r8(i2c, addr, RANGE_MM_HI)
        lo=_vl_r8(i2c, addr, RANGE_MM_LO)
        d=(hi<<8)|lo
        return None if d in (0,255,8190,8191) else d
    except:
        return None


# -------------------------------------------------------
# 9) Wind Speed Sensor
# -------------------------------------------------------
WIND_PIN=4
wind_pin=Pin(WIND_PIN,Pin.IN,Pin.PULL_UP)
wind_count=0
last_wind_ms=ticks_ms()

def _wind_irq(pin):
    global wind_count
    wind_count+=1

wind_pin.irq(trigger=Pin.IRQ_FALLING, handler=_wind_irq)

def read_wind():
    global wind_count, last_wind_ms
    now=ticks_ms()
    dt=ticks_diff(now,last_wind_ms)/1000
    if dt<=0:
        return 0.0
    pulses=wind_count
    wind_count=0
    last_wind_ms=now
    speed=pulses/dt
    return speed


# -------------------------------------------------------
# 10) Printing table formatting
# -------------------------------------------------------
def fmt(x,w=7,p=2):
    if x is None: 
        return "   ---  "
    try:
        return ("{:"+str(w)+"."+str(p)+"f}").format(float(x))
    except:
        return "   ---   "


def show_table(Ta,Tw2,UV,dist,mass,IR,Lux,model):
    print("+--------------------------------------------------------------------------+")
    print("| Model | T_air | T_w2 |  UV | Distmm | Mass |   IR   |   Lux   |")
    print("+--------------------------------------------------------------------------+")
    print("| {:>5} | {} | {} | {} | {} | {} | {} | {} |".format(
        model,
        fmt(Ta), fmt(Tw2),
        fmt(UV,5,0),
        fmt(dist,6,0),
        fmt(mass,6,2),
        fmt(IR,6,0),
        fmt(Lux,7,1)
    ))
    print("+--------------------------------------------------------------------------+\n")


# -------------------------------------------------------
# 11) Main loop every 30 seconds
# -------------------------------------------------------
PERIOD = 30

while True:

    # ----- MODEL A -----
    Ta,_   = safe_read_sht(sht_air_A)
    Tw2,_  = safe_read_sht(sht_w2_A)
    UV, UVI = ltr_read_uv(i2c_A1)
    IR, Lux = safe_tsl(tsl_A)
    dist    = vl_once_raw(i2c_A1)
    mass    = safe_mass(hxA)

    show_table(Ta,Tw2,UV,dist,mass,IR,Lux,"A")

    send_to_thingspeak(API_A, [
        Ta, Tw2, UV, dist, mass, IR, Lux
    ])


    # ----- MODEL B -----
    Tb,_   = safe_read_sht(sht_air_B)
    Tw2b,_ = safe_read_sht(sht_w2_B)
    UVb,UVIb = ltr_read_uv(i2c_B1)
    IRb, Luxb = safe_tsl(tsl_B)
    distb   = vl_once_raw(i2c_B1)
    massb   = safe_mass(hxB)

    show_table(Tb,Tw2b,UVb,distb,massb,IRb,Luxb,"B")

    send_to_thingspeak(API_B, [
        Tb, Tw2b, UVb, distb, massb, IRb, Luxb
    ])


    # ----- MODEL C -----
    Tc,_   = safe_read_sht(sht_air_C)
    Tw2c,_ = safe_read_sht(sht_w2_C)
    UVc,UVIc = ltr_read_uv(i2c_C1)
    IRc, Luxc = safe_tsl(tsl_C)
    distc    = vl_once_raw(i2c_C1)
    massc    = safe_mass(hxC)

    show_table(Tc,Tw2c,UVc,distc,massc,IRc,Luxc,"C")

    send_to_thingspeak(API_C, [
        Tc, Tw2c, UVc, distc, massc, IRc, Luxc
    ])


    # ----- Wind -----
    W = read_wind()
    send_to_thingspeak(API_W, [W])

    sleep(PERIOD)
