# An example of using resample() function with linear interpolation

import pandas as pd

import io

df = pd.read_csv(io.StringIO(
  'datetime,value\n'
  '"2018-04-08 00:28:52",10\n'
  '"2018-04-08 00:38:34",11\n'
  '"2018-04-08 00:48:57",9\n'
  '"2018-04-08 01:18:22",7\n'
))
df.datetime = pd.to_datetime(df.datetime)
new_df = df.resample('5min', on='datetime').mean().interpolate()
print(new_df.to_string())
