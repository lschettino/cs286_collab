import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

rgb = pd.DataFrame([np.random.randint(0, 255, 20), np.random.randint(0, 255, 20), np.random.randint(0, 255, 20)],
                   index=['r', 'g', 'b']).transpose()
print(rgb)

rgbtuples = [tuple(i) for i in rgb.values]
print(rgbtuples)

df = pd.DataFrame([np.tile(np.arange(1, 5), 5), np.repeat(np.arange(1, 6), 4), rgbtuples],
                  index=['vertical', 'horizontal', 'rgb']).transpose()
df_pivot = df.pivot(index='vertical', columns='horizontal', values='rgb')
print(df_pivot)

fig, ax = plt.subplots()

df_pivot_asarray = np.array([[list(tup) for tup in row] for row in df_pivot.to_numpy()])
print(df_pivot_asarray)

xlen = len(df_pivot.columns)
ylen = len(df_pivot.index)
ax.imshow(df_pivot_asarray, extent=[- 0.5, xlen - 0.5, -0.5, ylen - 0.5], origin='lower')
ax.set_xticks(range(xlen))
ax.set_xticklabels(df_pivot.columns)
ax.set_yticks(range(ylen))
ax.set_yticklabels(df_pivot.index)
ax.invert_yaxis() # seaborn shows the first row at the top
plt.show()