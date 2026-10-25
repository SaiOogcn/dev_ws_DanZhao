"""Explicit navigation profiles; baseline remains available for comparisons."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Tuning:
    name: str='balanced'
    sprint: bool=False
    maximum_speed: float=1.25
    ring_speed: float=.65
    channel_speed: float=.65
    lateral_acceleration: float=.85
    ring_lateral_acceleration: float=.50
    braking: float=.60
    steering_rate: float=1.5
    steering_acceleration: float=5.
    # Initial response coefficients came from the development trial. They are
    # not a calibration of the physical car. Override after identification.
    forward_gain: float=.896
    forward_damping: float=20.90
    forward_frequency2: float=176.84
    reverse_gain: float=1.180
    reverse_damping: float=3.29
    reverse_frequency2: float=16.42
    speed_time_constant: float=.16
    speed_acceleration: float=1.6


def tuning_for(name='balanced'):
    if name=='balanced':return Tuning()
    if name=='sprint':return Tuning('sprint',True,1.8,1.7,1.8,4.5,4.0,1.3,2.2,8.)
    raise ValueError('navigation_profile must be balanced or sprint')
