from dataclasses import dataclass, field
from typing import Any

@dataclass(order=True)
class DataServices(object):
    """Performs analysis on data."""
    priority: int
    item: Any=field(compare=False)

#Swoll Gainz wuz heer