#!/usr/bin/env python3
import numpy as np

import laika.raw_gnss as raw
from cereal import messaging
from laika import AstroDog, helpers
from laika.gps_time import GPSTime
from laika.raw_gnss import read_raw_ublox


class Processor():
  def __init__(self):
    constellations = ['GPS', 'GLONASS']
    self.dog = AstroDog(valid_const=constellations)

  def process(self, measurements):
    raw.process_measurements(measurements, self.dog)
    # measurements_by_epoch = raw.group_measurements_by_epoch(measurements)
    # measurements_by_satellite = raw.group_measurements_by_sat(measurements)
    # for meas_epoch in measurements_by_epoch[::10]:
    #   processed = raw.process_measurements(meas_epoch, self.dog)
    # processes_meas = process_measurements(new_meas, self.dog)

    # measurements.append(processes_meas)

    # wls_estimate = calc_pos_fix(processes_meas)

    # est_pos = wls_estimate[0][:3]

  def main(self):
    sm = messaging.SubMaster(['sensorEvents', 'gpsLocationExternal', 'ubloxGnss'])  # ['controlsState', 'carEvents', 'managerState'])
    measurements = []
    measurements_t = []
    use_offline = False
    live = True
    if use_offline:
      if live:
        example_data = np.load('../../laika_repo/examples/example_data/live_gnss_ublox/value')
        measurements = [read_raw_ublox(m_arr) for m_arr in example_data]
      else:
        example_data = np.load('../../laika_repo/examples/example_data/raw_gnss_ublox/value')
        measurements = [raw.normal_meas_from_array(m_arr) for m_arr in example_data]

      # lets limit this to GPS satellite for the sake of simplicity
      measurements = [m for m in measurements if helpers.get_constellation(m.prn) == 'GPS']

      # we organize the measurements by epoch and by satellite for easy plotting
      # measurements_by_epoch = raw.group_measurements_by_epoch(measurements)
      # measurements_by_satellite = raw.group_measurements_by_sat(measurements)
      self.process(measurements)
    else:
      while True:
        # qcomGnss?
        sm.update()
        # print('ubloxGnss', sm['ubloxGnss'])
        if sm.updated['ubloxGnss']:
          # print("sm['ubloxGnss'].measurementReport", sm['ubloxGnss'].measurementReport)

          print(sm['ubloxGnss'])
          # Can also be ephephines
          if sm['ubloxGnss'].measurementReport:
            report = sm['ubloxGnss']
            report = report.measurementReport

            # With internet: uses dog
            if len(report.measurements) > 0:
              new_meas = read_raw_ublox(report)
              new_meas = [m for m in new_meas if helpers.get_constellation(m.prn) == 'GPS']
              measurements.extend(new_meas)
              self.process(measurements)

              recv_time = GPSTime(report.gpsWeek, report.rcvTow)
              print(recv_time.as_datetime())
              measurements_t.append(recv_time.as_datetime())


if __name__ == "__main__":
  p = Processor()
  p.main()
