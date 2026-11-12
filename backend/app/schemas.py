import math
from pydantic import BaseModel, field_validator


class MoveRequest(BaseModel):
    target: tuple[float, float]
    max_velocity: float

    @field_validator('target')
    @classmethod
    def valid_target(cls, value):
        if len(value) != 2 or any(not math.isfinite(v) or abs(v) > 1.57 for v in value):
            raise ValueError('target must contain two finite angles within ±1.57 rad')
        return value

    @field_validator('max_velocity')
    @classmethod
    def valid_speed(cls, value):
        if not math.isfinite(value) or not 0 < value <= 0.5:
            raise ValueError('max_velocity must be within (0, 0.5] rad/s')
        return value


class EStopRequest(BaseModel):
    engage: bool
