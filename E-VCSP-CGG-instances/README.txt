Baseline charging instances

The 112 TXT files describe charging for fixed routes from the AP benchmark.
Each file is self-contained. Periods are numbered from 1 and last 5 minutes.

HORIZON          Number of charging periods.
CHARGERS         Number of identical chargers.
GRID_CAPACITY    Maximum combined power in any period.
MAX_ADJUSTMENTS  Maximum power changes during one vehicle's session.
POWER_LEVELS     Positive charging powers, in increasing order.
VEHICLES         Number of vehicle rows that follow.

Each vehicle row is: id  first_period  last_period  required_power
Both window endpoints are inclusive. required_power is the minimum sum of
power over occupied periods, already adjusted for the charging curve; it is
not kWh. GRID_CAPACITY and POWER_LEVELS use the same unit (1 unit = 0.6 kW).
