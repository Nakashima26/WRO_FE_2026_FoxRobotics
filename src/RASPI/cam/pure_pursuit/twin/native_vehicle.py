"""Native double-precision implementation of Vehicle; Python remains the reference."""
import ctypes as ct
from .vehicle import Vehicle
from .params import max_wheel_deg
from ..native import get_kernel

_FIELDS = ('x','y','heading_deg','v','servo_deg','wheel_servo_deg','v_wheel','sliding','wheel_lag_deg','last_wheel_deg')

class NativeVehicle(Vehicle):
    def __post_init__(self):
        super().__post_init__()
        self._state = (ct.c_double * 14)(*(getattr(self,k) for k in _FIELDS))
        st,m,v = self.params.steering,self.params.motor,self.params.vehicle
        vals=[st.servo_max_deg,st.servo_min_deg,st.deadband_deg,st.slew_deg_per_s,st.backlash_deg,st.lag_tau_s,st.servo_center_deg,st.gain_right,st.gain_left,st.wheel_max_right_deg,st.wheel_max_left_deg,m.pwm0,m.brake_lock_speed_mm_s,m.brake_slide_decel_mm_s2,m.brake_decel_mm_s2,m.brake_wheel_tau_s,m.coast_decel_mm_s2,m.pwm0_rev is not None,m.k_rev_mm_s_per_pwm,m.pwm0_rev,m.k_mm_s_per_pwm,max_wheel_deg(st),m.scrub_coeff,m.tau_coast_s,m.tau_drive_s,m.a_max_rev_mm_s2,m.a_max_mm_s2,m.a_max_rev_mm_s2 is not None,m.rev_spin_tau_s,m.reverse_wheel_scale,v.wheelbase_mm,m.a_lat_left_mm_s2,m.a_lat_right_mm_s2]
        vals += [len(st.table_right),len(st.table_left)]
        vals += [x for pair in st.table_right for x in pair]
        vals += [x for pair in st.table_left for x in pair]
        self._p=(ct.c_double*len(vals))(*(0 if x is None else x for x in vals))
        self._fn=get_kernel('fox_twin_vehicle',[ct.c_void_p,ct.c_void_p],None)
        if self._fn is None: raise RuntimeError('native vehicle unavailable')

    def _integrate_substep(self,dt,servo_cmd_deg,pwm,direction):
        s=self._state
        s[10]=dt; s[11]=servo_cmd_deg; s[12]=pwm; s[13]=direction
        self._fn(s,self._p)

# During dataclass initialization fields live in __dict__; after initialization
# descriptors expose the native state without copying all fields each substep.
def _field(i,k):
    def get(self):
        state=self.__dict__.get('_state')
        return state[i] if state is not None else self.__dict__[k]
    def set(self,value):
        state=self.__dict__.get('_state')
        if state is None:self.__dict__[k]=value
        else:state[i]=value
    return property(get,set)
for _i,_k in enumerate(_FIELDS):setattr(NativeVehicle,_k,_field(_i,_k))
