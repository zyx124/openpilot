#!/usr/bin/env python3

import pickle

from cereal import messaging


def main():
  sm = messaging.SubMaster(['sensorEvents', 'gpsLocationExternal', 'ubloxGnss'])  # ['controlsState', 'carEvents', 'managerState'])
  raw_reports = []
  raw_ubloxGnss = []
  raw_ephemeris = []
  amount = 20
  filename = f'ephemeris_{amount}.pickle'
  while True:
    sm.update()
    if sm.updated['ubloxGnss']:
      if sm['ubloxGnss'].which == 'ephemeris':
        raw_ephemeris.append(sm['ubloxGnss'])
        print("added ephemeris", len(raw_ephemeris))
        if len(raw_ephemeris) >19:
          break
      if sm['ubloxGnss'].which == 'measurementReport':
        # Can also be 'measurementReport', 'ephemeris', 'ionoData', 'hwStatus', 'hwStatus2'
        raw_ubloxGnss.append(sm['ubloxGnss'])

        report = sm['ubloxGnss'].measurementReport

        # With internet: uses dog
        if len(report.measurements) > 0:
          # new_meas = read_raw_ublox(report)
          # new_meas = [m for m in new_meas if helpers.get_constellation(m.prn) == 'GPS']
          print("Incoming", len(raw_reports))
          raw_reports.append(report)
          # if len(raw_reports) > amount:
          #   break

  with open(filename, 'wb') as handle:
    # pickle.dump({'raw_ubloxGnss': raw_ubloxGnss, 'raw_reports': raw_reports}, handle, protocol=pickle.HIGHEST_PROTOCOL)
    pickle.dump({'raw_ephemeris': raw_ephemeris}, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == "__main__":
  main()
