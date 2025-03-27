# drivebrain state estimator
role: handles the population / management of the internal state. Will also handle running of estimators that populate other internal state

## time-critical state inputs

The drivebrain receives data from VCF and VCR in several forms, including from CAN and ethernet as well as forwarded CAN messages from the inverters.

### CAN messages
from VCF:
- front suspension data
- pedals data

from VCR:
- rear suspension data
    - `REAR_SUSPENSION`
- forwarded inverter messages