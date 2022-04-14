import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('dataset-only-input-1.csv')

date_time = pd.to_datetime(df.pop('t'), unit='s', origin='unix')

plot_cols = ['x_acc', 'y_acc', 'z_acc', 'x_gyro', 'y_gyro', 'z_gyro', 'x_mag', 'y_mag', 'z_mag']
plot_features = df[plot_cols]
plot_features.index = date_time
plot_features.plot(subplots=True)

plt.show()
